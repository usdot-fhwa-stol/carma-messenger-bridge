#include "mosaic_adapter.hpp"
#include "mosaic_client.hpp"

MosaicAdapter::MosaicAdapter() : Node("mosaic_adapter"), mosaic_client_() {

    this->declare_parameter<std::string>("ip_address", "172.2.0.2");
    this->declare_parameter<std::string>("host_ip", "172.2.0.3");
    this->declare_parameter<bool>("enable_registration", false);
    this->declare_parameter<bool>("enable_vehicle_status", false);
    this->declare_parameter<int>("registration_port_remote", 6000);
    this->declare_parameter<int>("registration_port_local", 4001);
    this->declare_parameter<int>("vehicle_status_port_remote", 7000);
    this->declare_parameter<int>("vehicle_status_port_local", 4002);

    // Get parameters and directly populate the config structure
    this->get_parameter("ip_address", config_.ip_address);
    this->get_parameter("enable_registration", config_.enable_registration);
    this->get_parameter("enable_vehicle_status", config_.enable_vehicle_status);
    
    // Assign the port values directly to the config structure with type casting
    int temp_port;
    this->get_parameter("registration_port_remote", temp_port);
    config_.registration_port_remote = static_cast<unsigned short>(temp_port);
    this->get_parameter("registration_port_local", temp_port);
    config_.registration_port_local = static_cast<unsigned short>(temp_port);
    this->get_parameter("vehicle_status_port_remote", temp_port);
    config_.vehicle_status_port_remote = static_cast<unsigned short>(temp_port);
    this->get_parameter("vehicle_status_port_local", temp_port);
    config_.vehicle_status_port_local = static_cast<unsigned short>(temp_port);

    // Setup publishers (example usage)
    gps_publisher_ = this->create_publisher<gps_msgs::msg::GPSFix>("vehicle_pose", 10);
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("velocity", 10);
    time_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/sim_clock", 10);
}

MosaicAdapter::~MosaicAdapter() {
    // Clean up resources
}

void MosaicAdapter::on_vehicle_status_received_handler(const std::array<double, 3>& pose, const std::array<double, 3>& twist) {
    gps_msgs::msg::GPSFix gps_msg;
    geometry_msgs::msg::TwistStamped twist_msg;

    gps_msg.latitude = pose[0];  
    gps_msg.longitude = pose[1]; 
    gps_msg.altitude = pose[2];  

    RCLCPP_INFO(this->get_logger(), 
                "GPSFix Message - Lat: %f, Lon: %f, Alt: %f", 
                gps_msg.latitude, gps_msg.longitude, gps_msg.altitude);

    twist_msg.twist.linear.x = twist[0];
    twist_msg.twist.linear.y = twist[1];
    twist_msg.twist.linear.z = twist[2]; 

    twist_msg.header.stamp = this->now();

    RCLCPP_INFO(this->get_logger(), 
                "TwistStamped Message - Linear: x=%f, y=%f, z=%f", 
                twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z);

    gps_publisher_->publish(gps_msg);
    twist_publisher_->publish(twist_msg);
}

void MosaicAdapter::on_time_received_handler(unsigned long timestamp)
{
    rosgraph_msgs::msg::Clock time_now;

    // A script to validate time synchronization of tools in CDASim currently relies on the following
    // log line. TODO: This line is meant to be removed in the future upon completion of this work:
    // https://github.com/usdot-fhwa-stol/carma-analytics-fotda/pull/43
    auto chrono_time = std::chrono::system_clock::now();
    auto epoch = chrono_time.time_since_epoch();
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
    auto time_to_milli = static_cast<int>(timestamp / 1e6);

    // Using ROS2's debug logging system
    RCLCPP_DEBUG(rclcpp::get_logger("MosaicClient"), 
                 "Simulation Time: %d where current system time is: %ld", 
                 time_to_milli, milliseconds.count());

    // Setting the ROS2 Clock message
    time_now.clock.sec = static_cast<int>(timestamp / 1e9);
    time_now.clock.nanosec = static_cast<uint32_t>(timestamp - time_now.clock.sec * 1e9);

    time_pub_->publish(time_now);
}
