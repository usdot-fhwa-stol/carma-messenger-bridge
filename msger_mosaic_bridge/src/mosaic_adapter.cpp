#include "mosaic_adapter.hpp"
#include "mosaic_client.hpp"

MosaicAdapter::MosaicAdapter() : Node("mosaic_adapter"), mosaic_client_() {

    this->declare_parameter<std::string>("ip_address", "172.2.0.2");
    this->declare_parameter<std::string>("host_ip", "172.2.0.3")
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
    gps_publisher_ = this->create_publisher<gps_msgs::msg::GPSFix>("gps_topic", 10);
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist_topic", 10);

}

MosaicAdapter::~MosaicAdapter() {
    // Clean up resources
}



void MosaicAdapter::on_message_received(const std::vector<uint8_t>& data) {
    gps_msgs::msg::GPSFix gps_msg;
    geometry_msgs::msg::TwistStamped twist_msg;
    
    gps_publisher_->publish(gps_msg);
    twist_publisher_->publish(twist_msg);
}

void parse_data_to_gps_and_twist(const std::vector<uint8_t>& data, gps_msgs::msg::GPSFix& gps, geometry_msgs::msg::TwistStamped& twist) {
}
