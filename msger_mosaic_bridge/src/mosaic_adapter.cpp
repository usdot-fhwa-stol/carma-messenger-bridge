/*
 * Copyright (C) 2024 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
*/


#include "mosaic_adapter.hpp"
#include "mosaic_client.hpp"
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/schema.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

MosaicAdapter::MosaicAdapter() : Node("mosaic_adapter"), mosaic_client_() {

    this->declare_parameter<std::string>("/vehicle_id", "default_vehicle_id");
    this->declare_parameter<std::string>("role_id", "msg_veh_1");
    this->declare_parameter<std::string>("cdasim_ip_address", "127.0.0.1");
    this->declare_parameter<std::string>("host_ip", "127.0.0.1");
    this->declare_parameter<bool>("enable_registration", true);
    this->declare_parameter<bool>("enable_vehicle_status", true);
    this->declare_parameter<int>("registration_port_remote", 6000);
    this->declare_parameter<int>("registration_port_local", 4001);
    this->declare_parameter<int>("vehicle_status_port_remote", 7001);
    this->declare_parameter<int>("vehicle_status_port_local", 4002);
    this->declare_parameter<int>("siren_and_light_status_port_remote", 8001);
    this->declare_parameter<int>("siren_and_light_status_port_local", 4003);

    this->get_parameter("/vehicle_id", config_.vehicle_id);
    this->get_parameter("role_id", config_.role_id);
    this->get_parameter("cdasim_ip_address", config_.cdasim_ip_address);
    this->get_parameter("enable_registration", config_.enable_registration);
    this->get_parameter("enable_vehicle_status", config_.enable_vehicle_status);
    
    int temp_port;
    this->get_parameter("registration_port_remote", temp_port);
    config_.registration_port_remote = static_cast<unsigned short>(temp_port);
    this->get_parameter("registration_port_local", temp_port);
    config_.registration_port_local = static_cast<unsigned short>(temp_port);

    this->get_parameter("vehicle_status_port_remote", temp_port);
    config_.vehicle_status_port_remote = static_cast<unsigned short>(temp_port);
    this->get_parameter("vehicle_status_port_local", temp_port);
    config_.vehicle_status_port_local = static_cast<unsigned short>(temp_port);

    this->get_parameter("siren_and_light_status_port_remote", temp_port);
    config_.siren_and_light_status_port_remote = static_cast<unsigned short>(temp_port);
    this->get_parameter("siren_and_light_status_port_local", temp_port);
    config_.siren_and_light_status_port_local = static_cast<unsigned short>(temp_port);

    // Log all configuration data
    RCLCPP_INFO(this->get_logger(),
                "MosaicAdapter Configuration:\n"
                " - vehicle_id: %s\n"
                " - role_id: %s\n"
                " - cdasim_ip_address: %s\n"
                " - messenger_ip_address: %s\n"
                " - enable_registration: %s\n"
                " - enable_vehicle_status: %s\n"
                " - registration_port_remote: %d\n"
                " - registration_port_local: %d\n"
                " - vehicle_status_port_remote: %d\n"
                " - vehicle_status_port_local: %d\n"
                " - siren_and_light_status_port_remote: %d\n"
                " - siren_and_light_status_port_local: %d\n",
                config_.vehicle_id.c_str(),
                config_.role_id.c_str(),
                config_.cdasim_ip_address.c_str(),
                config_.messenger_ip_address.c_str(),
                config_.enable_registration ? "true" : "false",
                config_.enable_vehicle_status ? "true" : "false",
                config_.registration_port_remote,
                config_.registration_port_local,
                config_.vehicle_status_port_remote,
                config_.vehicle_status_port_local,
                config_.siren_and_light_status_port_remote,
                config_.siren_and_light_status_port_local);

    boost::system::error_code ec;
    bool init_successful = mosaic_client_.initialize(config_, ec);

    if (!init_successful) {
        if (ec) {
            RCLCPP_ERROR(this->get_logger(), "MosaicClient initialization failed with error: %s", ec.message().c_str());
            throw std::runtime_error("Failed to initialize MosaicClient: " + ec.message());
        } else {
            RCLCPP_ERROR(this->get_logger(), "MosaicClient initialization failed due to an unknown issue.");
            throw std::runtime_error("Failed to initialize MosaicClient due to an unknown issue.");
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "MosaicClient initialized successfully.");
    }

    gps_pub_ = this->create_publisher<gps_msgs::msg::GPSFix>("vehicle_pose", 10);
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("velocity", 10);
    time_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/sim_clock", 10);

    mosaic_client_.onTimeReceived.connect([this](unsigned long timestamp) {this->on_time_received_handler(timestamp); });
    mosaic_client_.onVehPoseReceived.connect([this](const std::array<double, 3>& pose) {
        this->on_vehicle_pose_received_handler(pose);
    });
    mosaic_client_.onVehTwistReceived.connect([this](const std::array<double, 3>& twist) {
        this->on_vehicle_twist_received_handler(twist);
    });
    mosaic_client_.onSirenAndLightStatuReceived.connect([this](bool siren_active, bool light_active){
        this->on_siren_and_light_status_recieved_handler(siren_active, light_active);
    });


}

void MosaicAdapter::initialize(){
    std::string handshake_msg = compose_handshake_msg(config_.role_id, 
                                                      config_.vehicle_status_port_local, 
                                                      config_.registration_port_local, 
                                                      config_.cdasim_ip_address);
    broadcast_handshake_msg(handshake_msg);
}

void MosaicAdapter::shutdown()
{
    RCLCPP_INFO(this->get_logger(), "Shutdown signal received");
    mosaic_client_.close();
}

void MosaicAdapter::on_vehicle_pose_received_handler(const std::array<double, 3>& pose) {
    RCLCPP_DEBUG(this->get_logger(), 
                "Received Pose - Lat: %f, Lon: %f, Alt: %f", 
                pose[0], pose[1], pose[2]);

    gps_msgs::msg::GPSFix gps_msg;
    gps_msg.latitude = pose[0];  
    gps_msg.longitude = pose[1]; 
    gps_msg.altitude = pose[2];  

    RCLCPP_DEBUG(this->get_logger(), 
                "GPSFix Message - Lat: %f, Lon: %f, Alt: %f", 
                gps_msg.latitude, gps_msg.longitude, gps_msg.altitude);

    gps_pub_->publish(gps_msg);
}


void MosaicAdapter::on_vehicle_twist_received_handler(const std::array<double, 3>& twist) {
    RCLCPP_DEBUG(this->get_logger(), 
            "Received Twist - x: %f, y: %f, z: %f", 
            twist[0], twist[1], twist[2]);

    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.twist.linear.x = twist[0];
    twist_msg.twist.linear.y = twist[1];
    twist_msg.twist.linear.z = twist[2]; 

    twist_msg.header.stamp = this->now();

    RCLCPP_DEBUG(this->get_logger(), 
                "TwistStamped Message - Linear: x=%f, y=%f, z=%f", 
                twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z);

    twist_pub_->publish(twist_msg);
}


void MosaicAdapter::on_time_received_handler(unsigned long timestamp)
{
    rosgraph_msgs::msg::Clock time_now;

    auto chrono_time = std::chrono::system_clock::now();
    auto epoch = chrono_time.time_since_epoch();
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
    auto time_to_milli = static_cast<int>(timestamp / 1e6);

    RCLCPP_DEBUG(rclcpp::get_logger("MosaicClient"), 
                 "Simulation Time: %d where current system time is: %ld", 
                 time_to_milli, milliseconds.count());

    time_now.clock.sec = static_cast<int>(timestamp / 1e9);
    time_now.clock.nanosec = static_cast<uint32_t>(timestamp - time_now.clock.sec * 1e9);

    time_pub_->publish(time_now);
}

void MosaicAdapter::on_siren_and_light_status_recieved_handler(bool siren_active, bool light_active){
    uint8_t status_code = 0;
    if (!siren_active && !light_active) {
        status_code = SIRENS_AND_LIGHTS_INACTIVE;
    } else if (siren_active && !light_active) {
        status_code = ONLY_SIRENS_ACTIVE;
    } else if (!siren_active && light_active) {
        status_code = ONLY_LIGHTS_ACTIVE;
    } else if (siren_active && light_active) {
        status_code = SIRENS_AND_LIGHTS_ACTIVE;
    }
    
    auto message = std::make_shared<std::vector<uint8_t>>(1, status_code);
    mosaic_client_.send_siren_and_light_message(message);
}


std::string MosaicAdapter::compose_handshake_msg(const std::string& role_id, 
                                                 int veh_status_port, 
                                                 int time_port, 
                                                 const std::string& ip)
{
    // document is the root of a json message
	rapidjson::Document document;

	// define the document as an object rather than an array
	document.SetObject();

	// create a rapidjson array type with similar syntax to std::vector
	rapidjson::Value array(rapidjson::kArrayType);

	// must pass an allocator when the object may need to allocate memory
	rapidjson::Document::AllocatorType& allocator = document.GetAllocator();

    rapidjson::Value roletextPart;
	roletextPart.SetString(role_id.c_str(), allocator);
    document.AddMember("sumoVehicleRole", roletextPart, allocator);

    rapidjson::Value iptextPart;
	iptextPart.SetString(ip.c_str(), allocator);
    document.AddMember("rxIpAddress", iptextPart, allocator);

    rapidjson::Value porttextPart;
	porttextPart.SetInt(veh_status_port);
    document.AddMember("rxVehicleStatusPort", porttextPart, allocator);

    rapidjson::Value portTimePart;
	portTimePart.SetInt(time_port);
    document.AddMember("rxTimeSyncPort", portTimePart, allocator);

    rapidjson::StringBuffer strbuf;
	rapidjson::Writer<rapidjson::StringBuffer> writer(strbuf);
	document.Accept(writer);

    std::string strbufstring = strbuf.GetString();

    return strbufstring;
}

void MosaicAdapter::broadcast_handshake_msg(const std::string& msg_string)
{
    RCLCPP_DEBUG(this->get_logger(), "Attempting to broadcast_handshake_msg: %s", msg_string.c_str());
    auto msg_vector = std::vector<uint8_t>(msg_string.begin(), msg_string.end());
    std::shared_ptr<std::vector<uint8_t>> message_content = std::make_shared<std::vector<uint8_t>>(std::move(msg_vector));

    bool success = mosaic_client_.send_registration_message(message_content);
    RCLCPP_DEBUG(this->get_logger(), "ip_address: %s", config_.cdasim_ip_address.c_str());
    RCLCPP_DEBUG(this->get_logger(), "registration_port_local: %d", config_.registration_port_local);
    RCLCPP_DEBUG(this->get_logger(), "vehicle_status_port_local: %d", config_.vehicle_status_port_local);

    if (!success) {
        RCLCPP_WARN(this->get_logger(), "Handshake Message send failed");
    }
    else {
        RCLCPP_DEBUG(this->get_logger(), "Handshake Message sent successfully");
    }
}
