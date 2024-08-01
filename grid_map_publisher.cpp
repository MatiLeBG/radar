#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <math.h>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <opencv2/opencv.hpp>

#include "navtech_msgs/msg/radar_configuration_msg.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "grid_map_publisher.h"
#include "net_conversion.h"
#include "Endpoint.h"

#include "Colossus_TCP_client.h"
#include "Time_utils.h"
#include "Colossus_protocol.h"
#include "configurationdata.pb.h"
#include "Protobuf_helpers.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

using Navtech::Networking::Colossus_protocol::TCP::Client;
using Navtech::Networking::Colossus_protocol::TCP::Message;

using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;
using namespace Navtech::Unit;
using namespace Navtech::Networking;
using namespace Navtech::Networking::Colossus_protocol::TCP;

bool Grid_map_publisher::rotated_once(Azimuth_num azimuth)
{
    static bool has_rotated_once { };
    static Azimuth_num prev { 0 };
    
    if (has_rotated_once) return true;
    if (azimuth <= prev) has_rotated_once = true;
    prev = azimuth;

    return has_rotated_once;
}


bool Grid_map_publisher::completed_full_rotation(Azimuth_num azimuth)
{
    if (!rotated_once(azimuth)) return false;

    bool has_completed_rotation { false };
    static Azimuth_num prev { };

    if (azimuth <= prev) has_completed_rotation = true;
    prev = azimuth;

    return has_completed_rotation;
}


Grid_map_publisher::Grid_map_publisher():Node{ "grid_map_publisher" }
{
    declare_parameter("radar_ip", "");
    declare_parameter("radar_port", 0);
    declare_parameter("start_azimuth", 0);
    declare_parameter("end_azimuth", 0);
    declare_parameter("start_bin", 0);
    declare_parameter("end_bin", 0);
    declare_parameter("power_threshold", 0);
    declare_parameter("azimuth_offset", 0);
    declare_parameter("combined_distance_offset", 0.0);
    declare_parameter("combined_distance_scale_factor", 0.0);
    declare_parameter("x_distance_offset", 0.0);
    declare_parameter("y_distance_offset", 0.0);

    radar_ip = get_parameter("radar_ip").as_string();
    radar_port = get_parameter("radar_port").as_int();
    start_azimuth = get_parameter("start_azimuth").as_int();
    end_azimuth = get_parameter("end_azimuth").as_int();
    start_bin = get_parameter("start_bin").as_int();
    end_bin = get_parameter("end_bin").as_int();
    power_threshold = get_parameter("power_threshold").as_int();
    azimuth_offset = get_parameter("azimuth_offset").as_int();
    combined_distance_offset = get_parameter("combined_distance_offset").as_double();
    combined_distance_scale_factor = get_parameter("combined_distance_scale_factor").as_double();
    x_distance_offset = get_parameter("x_distance_offset").as_double();
    y_distance_offset = get_parameter("y_distance_offset").as_double();

    // Set up the radar client
    //
    Endpoint server_addr { Navtech::Networking::IP_address(radar_ip), radar_port };
    radar_client = Navtech::allocate_owned<Navtech::Networking::Colossus_protocol::TCP::Client>(
        server_addr
    );


    radar_client->set_handler(
        Type::fft_data, 
        std::bind(&Grid_map_publisher::fft_data_handler, this, std::placeholders::_1, std::placeholders::_2)
    );
    radar_client->set_handler(
        Type::configuration, 
        std::bind(&Grid_map_publisher::configuration_data_handler, this, std::placeholders::_1, std::placeholders::_2)
    );

    rclcpp::QoS qos_radar_configuration_publisher(radar_configuration_queue_size);
    qos_radar_configuration_publisher.reliable();

    configuration_data_publisher =
    Node::create_publisher<navtech_msgs::msg::RadarConfigurationMsg>(
        "radar_data/configuration_data",
        qos_radar_configuration_publisher
    );

    rclcpp::QoS qos_point_cloud_publisher(radar_point_cloud_queue_size);
    qos_point_cloud_publisher.reliable();

    grid_map_publisher =
    Node::create_publisher<sensor_msgs::msg::PointCloud2>(
        "radar_data/grid_map",
        qos_point_cloud_publisher
    );

    // setting up subsbscriber to barakuda odometry
    odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "/main_odometry", 10,
        std::bind(&Grid_map_publisher::odometry_callback, this, std::placeholders::_1));
}

void Grid_map_publisher::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Position
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;

    // Orientation
    ox = msg->pose.pose.orientation.x;
    oy = msg->pose.pose.orientation.y;
    oz = msg->pose.pose.orientation.z;
    ow = msg->pose.pose.orientation.w;

    // Linear speed
    lx = msg->twist.twist.linear.x;
    ly = msg->twist.twist.linear.y;
    lz = msg->twist.twist.linear.z;

    // Angular speed
    ax = msg->twist.twist.angular.x;
    ay = msg->twist.twist.angular.y;
    az = msg->twist.twist.angular.z;
}


Grid_map_publisher::~Grid_map_publisher()
{
    stop();
}


void Grid_map_publisher::start()
{
    radar_client->start();
}


void Grid_map_publisher::stop()
{
    radar_client->remove_handler(Type::configuration);
    radar_client->remove_handler(Type::fft_data);

    radar_client->send(Type::stop_fft_data);
    radar_client->stop();
}


void Grid_map_publisher::publish_point_cloud()
{
    std_msgs::msg::Header header;
    header.stamp = get_clock()->now();
    header.frame_id = "point_cloud";

    // Convertit GridMap en PointCloud2
    sensor_msgs::msg::PointCloud2 point_cloud_msg;
    grid_map::GridMapRosConverter::toPointCloud(gridMap, "height", point_cloud_msg);

    // Met à jour l'en-tête du message point cloud
    point_cloud_msg.header = header;

    // Publie le message
    grid_map_publisher->publish(point_cloud_msg);

    gridMap.clearAll();
    pointMap.clearAll();
    gridMap.add("intensity", 0.0);
    gridMap.add("height", 0.0);
    pointMap.add("num_points", 0.0);
}


void Grid_map_publisher::fft_data_handler(Client& radar_client [[maybe_unused]], Message& msg)
{
    using namespace Navtech::Networking;
    using namespace Navtech::Time::Monotonic;
    using Navtech::Protobuf::from_vector_into;

    auto fft  = msg.view_as<Colossus_protocol::TCP::FFT_data>();
    auto data = fft->to_vector();
    int azimuth_index = (int)(fft->azimuth() / (encoder_size / azimuth_samples));

    // To adjust radar start azimuth, for sake of visualisation
    // Note - this value will be different for every setup!
    // Values based on 0 angle of radar, and surrounding landscape
    int adjusted_azimuth_index = azimuth_index + azimuth_offset;
    if (adjusted_azimuth_index >= azimuth_samples) {
        adjusted_azimuth_index = adjusted_azimuth_index - azimuth_samples;
    }
    
    static bool is_first_open = true;
    std::ofstream bin_file;
    if (is_first_open) {
        bin_file.open("data_file.bin", std::ios::binary | std::ios::trunc);
        is_first_open = false; 
    } else {
        bin_file.open("data_file.bin", std::ios::binary | std::ios::app);
    }

    if (!bin_file) {
        RCLCPP_ERROR(Node::get_logger(), "Erreur d'ouverture du fichier binaire.");
        return;
    }
    
    if ((azimuth_index >= start_azimuth) && (azimuth_index < end_azimuth)) {
        
        for (unsigned bin_index = start_bin; bin_index < data.size(); bin_index++) {
            if ((bin_index >= start_bin) && (bin_index < end_bin)) {
                if (data[bin_index] > power_threshold) {
                    float bearing = static_cast<float>(adjusted_azimuth_index) / static_cast<float>(azimuth_samples) * 360.0;
                    float range = ((bin_index * bin_size / 10000.0) * range_gain * combined_distance_scale_factor) + range_offset + combined_distance_offset;
                    float intensity = static_cast<float>(data[bin_index]);

                    auto pos = Navtech::Polar::Coordinate(range, bearing).to_cartesian();
                    
                    grid_map::Index index;
                    gridMap.getIndex(grid_map::Position(pos.x, pos.y), index);
                    
                    float current_intensity = gridMap.at("intensity", index);
                    int num_points_in_cell = pointMap.at("num_points", index);
                    float new_intensity = (current_intensity * num_points_in_cell + data[bin_index]) / (num_points_in_cell + 1);
                    
                    gridMap.at("intensity", index) = new_intensity;
                    pointMap.at("num_points", index) = num_points_in_cell + 1;

                    bin_file.write(reinterpret_cast<const char*>(&range), sizeof(range));
                    bin_file.write(reinterpret_cast<const char*>(&bearing), sizeof(bearing));
                    bin_file.write(reinterpret_cast<const char*>(&intensity), sizeof(intensity));
                    
                    points_this_rotation += 1;
                }
            }
        }
    }


    bin_file.close();

    if (!completed_full_rotation(fft->azimuth())) {
        return;
    }

    rotation_count++;
    Grid_map_publisher::publish_point_cloud();
    points_this_rotation = 0;

    

    if (rotation_count >= config_publish_count) {

        int temp_azimuth_offset = get_parameter("azimuth_offset").as_int();
        if (temp_azimuth_offset > azimuth_samples) {
            RCLCPP_INFO(Node::get_logger(), "Azimuth offset of %i is invalid, must be less than or equal to %i", temp_azimuth_offset, azimuth_samples);
            RCLCPP_INFO(Node::get_logger(), "Setting azimuth offset to %i", azimuth_samples);
            set_parameter(rclcpp::Parameter("azimuth_offset", azimuth_samples));
        }
        else {
            azimuth_offset = temp_azimuth_offset;
        }

        int temp_start_azimuth = get_parameter("start_azimuth").as_int();
        if (temp_start_azimuth < 0 || temp_start_azimuth > azimuth_samples) {
            RCLCPP_INFO(Node::get_logger(), "Start azimuth of %i is invalid, must be between 0 and %i", temp_start_azimuth, azimuth_samples);
            RCLCPP_INFO(Node::get_logger(), "Setting start azimuth to %i", 0);
            set_parameter(rclcpp::Parameter("start_azimuth", 0));
        }
        else {
            start_azimuth = temp_start_azimuth;
        }

        int temp_end_azimuth = get_parameter("end_azimuth").as_int();
        if (temp_end_azimuth < 0 || temp_end_azimuth > azimuth_samples) {
            RCLCPP_INFO(Node::get_logger(), "End azimuth of %i is invalid, must be between 0 and %i", temp_end_azimuth, azimuth_samples);
            RCLCPP_INFO(Node::get_logger(), "Setting end azimuth to %i", azimuth_samples);
            set_parameter(rclcpp::Parameter("end_azimuth", azimuth_samples));
        }
        else {
            end_azimuth = temp_end_azimuth;
        }

        int temp_start_bin = get_parameter("start_bin").as_int();
        if (temp_start_bin < 0 || temp_start_bin > range_in_bins) {
            RCLCPP_INFO(Node::get_logger(), "Start bin of %i is invalid, must be between 0 and %i", temp_start_bin, range_in_bins);
            RCLCPP_INFO(Node::get_logger(), "Setting start bin to %i", 0);
            set_parameter(rclcpp::Parameter("start_bin", 0));
        }
        else {
            start_bin = temp_start_bin;
        }

        int temp_end_bin = get_parameter("end_bin").as_int();
        if (temp_end_bin < 0 || temp_end_bin > range_in_bins) {
            RCLCPP_INFO(Node::get_logger(), "End bin of %i is invalid, must be between 0 and %i", temp_end_bin, range_in_bins);
            RCLCPP_INFO(Node::get_logger(), "Setting end bin to %i", range_in_bins);
            set_parameter(rclcpp::Parameter("end_bin", range_in_bins));
        }
        else {
            end_bin = temp_end_bin;
        }

        int temp_power_threshold = get_parameter("power_threshold").as_int();
        if (temp_power_threshold < 0 || temp_power_threshold > std::numeric_limits<uint8_t>::max()) {
            RCLCPP_INFO(Node::get_logger(), "Power threshold of %i is invalid, must be between 0 and %i", temp_power_threshold, std::numeric_limits<uint8_t>::max());
            RCLCPP_INFO(Node::get_logger(), "Setting power threshold to %i", std::numeric_limits<uint8_t>::max() / 2);
            set_parameter(rclcpp::Parameter("power_threshold", power_threshold));
        }
        else {
            power_threshold = temp_power_threshold;
        }

        double temp_combined_distance_offset = get_parameter("combined_distance_offset").as_double();
        if (temp_combined_distance_offset < 0.0 || temp_combined_distance_offset > 1000000.0) {
            RCLCPP_INFO(Node::get_logger(), "Combined distance offset of %f is invalid, must be between %f and %f", temp_combined_distance_offset, 0.0, 1000000.0);
            RCLCPP_INFO(Node::get_logger(), "Setting combined distance offset to 0.0");
            set_parameter(rclcpp::Parameter("combined_distance_offset", 0.0));
        }
        else {
            combined_distance_offset = temp_combined_distance_offset;
        }

        double temp_combined_distance_scale_factor = get_parameter("combined_distance_scale_factor").as_double();
        if (temp_combined_distance_scale_factor < 0.0 || temp_combined_distance_scale_factor > 1000000.0) {
            RCLCPP_INFO(Node::get_logger(), "Combined distance scale factor of %f is invalid, must be between %f and %f", temp_combined_distance_scale_factor, 0.0, 1000000.0);
            RCLCPP_INFO(Node::get_logger(), "Setting combined distance scale factor to 0.0");
            set_parameter(rclcpp::Parameter("combined_distance_scale_factor", 0.0));
        }
        else {
            combined_distance_offset = temp_combined_distance_scale_factor;
        }

        double temp_x_distance_offset = get_parameter("x_distance_offset").as_double();
        if (temp_x_distance_offset < 0.0 || temp_x_distance_offset > 1000000.0) {
            RCLCPP_INFO(Node::get_logger(), "X distance offset of %f is invalid, must be between %f and %f", temp_x_distance_offset, 0.0, 1000000.0);
            RCLCPP_INFO(Node::get_logger(), "Setting X distance offset to 0.0");
            set_parameter(rclcpp::Parameter("x_distance_offset", 0.0));
        }
        else {
            x_distance_offset = temp_x_distance_offset;
        }

        double temp_y_distance_offset = get_parameter("y_distance_offset").as_double();
        if (temp_y_distance_offset < 0.0 || temp_y_distance_offset > 1000000.0) {
            RCLCPP_INFO(Node::get_logger(), "Y distance offset of %f is invalid, must be between %f and %f", temp_y_distance_offset, 0.0, 1000000.0);
            RCLCPP_INFO(Node::get_logger(), "Setting Y distance offset to 0.0");
            set_parameter(rclcpp::Parameter("y_distance_offset", 0.0));
        }
        else {
            y_distance_offset = temp_y_distance_offset;
        }

        configuration_data_publisher->publish(config_message);
        rotation_count = 0;
    }
}


void Grid_map_publisher::configuration_data_handler(Client& radar_client [[maybe_unused]], Message& msg)
{
    auto config   = msg.view_as<Configuration>();
    RCLCPP_INFO(Node::get_logger(), "Configuration Data Received");
    RCLCPP_INFO(Node::get_logger(), "Azimuth Samples: %i", config->azimuth_samples());
    RCLCPP_INFO(Node::get_logger(), "Encoder Size: %i", config->encoder_size());
    RCLCPP_INFO(Node::get_logger(), "Bin Size: %i", config->bin_size());
    RCLCPP_INFO(Node::get_logger(), "Range In Bins: : %i", config->range_in_bins());
    RCLCPP_INFO(Node::get_logger(), "Expected Rotation Rate: %i", config->rotation_speed());
    RCLCPP_INFO(Node::get_logger(), "Range Gain: %f", config->range_gain());
    RCLCPP_INFO(Node::get_logger(), "Range Offset: %f", config->range_offset());
    RCLCPP_INFO(Node::get_logger(), "Publishing Configuration Data");

    azimuth_samples = config->azimuth_samples();
    encoder_size = config->encoder_size();
    bin_size = config->bin_size();
    end_bin = config->range_in_bins();
    range_in_bins = config->range_in_bins();
    expected_rotation_rate = config->rotation_speed();
    range_gain = config->range_gain();
    range_offset = config->range_offset();
    config_message.header = std_msgs::msg::Header();
    config_message.header.stamp = get_clock()->now();
    config_message.header.frame_id = "point_cloud";
    config_message.azimuth_samples = config->azimuth_samples();
    config_message.encoder_size = config->encoder_size();
    config_message.bin_size = config->bin_size();
    config_message.range_in_bins = config->range_in_bins();
    config_message.expected_rotation_rate = config->rotation_speed();
    config_message.range_gain = config->range_gain();
    config_message.range_offset = config->range_offset();
    config_message.azimuth_offset = azimuth_offset;
    configuration_data_publisher->publish(config_message);

    RCLCPP_INFO(Node::get_logger(), "Starting point cloud publisher");
    RCLCPP_INFO(Node::get_logger(), "Start azimuth: %i", start_azimuth);
    RCLCPP_INFO(Node::get_logger(), "End azimuth: %i", end_azimuth);
    RCLCPP_INFO(Node::get_logger(), "Start bin: %i", start_bin);
    RCLCPP_INFO(Node::get_logger(), "End bin: %i", end_bin);
    RCLCPP_INFO(Node::get_logger(), "Power threshold: %i", power_threshold);
    RCLCPP_INFO(Node::get_logger(), "Azimuth offset: %i", azimuth_offset);
    RCLCPP_INFO(Node::get_logger(), "Combined distance offset: %f", combined_distance_offset);
    RCLCPP_INFO(Node::get_logger(), "Combined distance scale factor: %f", combined_distance_scale_factor);
    RCLCPP_INFO(Node::get_logger(), "X distance offset: %f", x_distance_offset);
    RCLCPP_INFO(Node::get_logger(), "Y distance offset: %f", y_distance_offset);

    grid_map_size = 2 * end_bin - 1;

    gridMap.setGeometry(grid_map::Length(grid_map_size * (bin_size / 10000), grid_map_size * (bin_size / 10000)), (bin_size / 10000));
    pointMap.setGeometry(grid_map::Length(grid_map_size * (bin_size / 10000), grid_map_size * (bin_size / 10000)), (bin_size / 10000));
    gridMap.add("intensity", 0.0);
    gridMap.add("height", 0.0);

    pointMap.add("num_points", 0);
    grid_map::Index center_index(grid_map_size / 2, grid_map_size / 2);

    radar_client.send(Type::start_fft_data);
}
