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


#include "mosaic_client.hpp"
#include <rapidjson/document.h>

MosaicClient::MosaicClient() {
    conn_manager_.onError.connect([this](const boost::system::error_code& err){conn_manager_error_ = err;});
}

MosaicClient::~MosaicClient() {
    try {
        close();
    } catch (...) {}
}

bool MosaicClient::initialize(const ConnectionConfig& config, boost::system::error_code &ec) {
    bool all_connected = true;

    all_connected &= conn_manager_.connect(ConnectionType::Registration, 
                                            config.cdasim_ip_address, 
                                            config.registration_port_remote, 
                                            config.registration_port_local,
                                            [this](const std::shared_ptr<const std::vector<uint8_t>>& data) { this->received_time(data); }, 
                                            ec, 
                                            registration_running_);

    all_connected &= conn_manager_.connect(ConnectionType::VehicleStatus, 
                                            config.cdasim_ip_address, 
                                            0, 
                                            config.vehicle_status_port_local,
                                            [this](const std::shared_ptr<const std::vector<uint8_t>>& data) { this->received_vehicle_status(data); }, 
                                            ec, 
                                            vehicle_status_running_);
                                            
    all_connected &= conn_manager_.connect(ConnectionType::SirenAndLightStatus, 
                                            config.messenger_ip_address, 
                                            config.siren_and_light_status_port_remote, 
                                            0,
                                            nullptr, //no need a listener
                                            ec, 
                                            siren_and_light_running_);
    all_connected &= conn_manager_.connect(ConnectionType::TrafficEvent,
                                            config.cdasim_ip_address,
                                            0,
                                            config.traffic_event_port_local,
                                            [this](const std::shared_ptr<const std::vector<uint8_t>>& data) { this->received_vehicle_status(data); }, 
                                            ec,
                                            traffic_event_running_);
    if (all_connected)
        RCLCPP_INFO(rclcpp::get_logger("MosaicClient"), "siren_and_light_status connection initialized successfully.");      

    

    return all_connected;
}

void MosaicClient::close() {
    if (registration_running_){
        RCLCPP_INFO(rclcpp::get_logger("MosaicClient"), "Stopping registration connection");  
        conn_manager_.close(ConnectionType::Registration, registration_running_);
    }
        
    if (vehicle_status_running_){
        RCLCPP_INFO(rclcpp::get_logger("MosaicClient"), "Stopping vehicle_status connection");  
        conn_manager_.close(ConnectionType::VehicleStatus, vehicle_status_running_);
    }
    if (siren_and_light_running_){
        RCLCPP_INFO(rclcpp::get_logger("MosaicClient"), "Stopping siren_and_light_status connection");  
        conn_manager_.close(ConnectionType::SirenAndLightStatus, siren_and_light_running_);
    }
}

bool MosaicClient::send_registration_message(const std::shared_ptr<std::vector<uint8_t>>& message) {
    bool isSent = conn_manager_.send_message(ConnectionType::Registration, message);
    if (conn_manager_error_.value())
        RCLCPP_ERROR(
            rclcpp::get_logger("MosaicClient"),
            "Failed to send registration message with error: %s (%d)", 
            conn_manager_error_.message().c_str(), 
            conn_manager_error_.value()
        );
    return isSent;
}

bool MosaicClient::send_siren_and_light_message(const std::shared_ptr<std::vector<uint8_t>>& message) {
    bool isSent = conn_manager_.send_message(ConnectionType::SirenAndLightStatus, message);
    if (conn_manager_error_.value())
        RCLCPP_ERROR(
            rclcpp::get_logger("MosaicClient"),
            "Failed to send siren_and_light_status message with error: %s (%d)", 
            conn_manager_error_.message().c_str(), 
            conn_manager_error_.value()
        );
    return isSent;
}

void MosaicClient::received_traffic_event(const std::shared_ptr<const std::vector<uint8_t>>& data) {
        // Convert data to JSON string
        std::string json_string(data->begin(), data->end());

        // Parse the JSON
        rapidjson::Document received_json;
        if (received_json.Parse(json_string.c_str()).HasParseError()) {
            RCLCPP_ERROR(rclcpp::get_logger("MosaicClient"), "Failed to parse JSON: %s", json_string.c_str());
            return;
        }

        double up_track = 0.0;
        double down_track = 0.0;
        double minimum_gap = 0.0;
        double advisory_speed = 0.0;

        // Extract the values
        if (received_json.HasMember("up_track") && received_json["up_track"].IsNumber()) {
            up_track = received_json["up_track"].GetDouble();
        } else {
            RCLCPP_WARN(rclcpp::get_logger("MosaicClient"), "Missing or invalid 'up_track' in JSON");
        }

        if (received_json.HasMember("down_track") && received_json["down_track"].IsNumber()) {
            down_track = received_json["down_track"].GetDouble();
        } else {
            RCLCPP_WARN(rclcpp::get_logger("MosaicClient"), "Missing or invalid 'down_track' in JSON");
        }

        if (received_json.HasMember("minimum_gap") && received_json["minimum_gap"].IsNumber()) {
            minimum_gap = received_json["minimum_gap"].GetDouble();
        } else {
            RCLCPP_WARN(rclcpp::get_logger("MosaicClient"), "Missing or invalid 'minimum_gap' in JSON");
        }

        if (received_json.HasMember("advisory_speed") && received_json["advisory_speed"].IsNumber()) {
            advisory_speed = received_json["advisory_speed"].GetDouble();
        } else {
            RCLCPP_WARN(rclcpp::get_logger("MosaicClient"), "Missing or invalid 'advisory_speed' in JSON");
        }

        // Log the extracted information
        RCLCPP_DEBUG(rclcpp::get_logger("MosaicClient"), 
                    "Received Traffic Event - Up Track: %f, Down Track: %f, Minimum Gap: %f, Advisory Speed: %f", 
                    up_track, down_track, minimum_gap, advisory_speed);

        onTrafficEventReceived(up_track, down_track, minimum_gap, advisory_speed);
}

void MosaicClient::received_vehicle_status(const std::shared_ptr<const std::vector<uint8_t>>& data) {

    std::string json_string(data->begin(), data->end());

    rapidjson::Document received_json;
    if (received_json.Parse(json_string.c_str()).HasParseError()) {
        RCLCPP_ERROR(rclcpp::get_logger("MosaicClient"), "Failed to parse JSON: %s", json_string.c_str());
        return;
    }

    std::array<double, 3> pose = {0.0, 0.0, 0.0};
    std::array<double, 3> twist = {0.0, 0.0, 0.0};
    bool siren_active = false;
    bool light_active = false;

    if (received_json.HasMember("vehicle_pose") && received_json["vehicle_pose"].IsObject()) {
        const auto& vehicle_pose = received_json["vehicle_pose"];
        if (vehicle_pose.HasMember("lat") && vehicle_pose["lat"].IsNumber() &&
            vehicle_pose.HasMember("lon") && vehicle_pose["lon"].IsNumber() &&
            vehicle_pose.HasMember("alt") && vehicle_pose["alt"].IsNumber()) {
            pose[0] = vehicle_pose["lat"].GetDouble();
            pose[1] = vehicle_pose["lon"].GetDouble();
            pose[2] = vehicle_pose["alt"].GetDouble();
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("MosaicClient"), "Invalid or missing fields in 'vehicle_pose'");
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("MosaicClient"), "Missing 'vehicle_pose' in JSON");
    }

    if (received_json.HasMember("vehicle_twist") && received_json["vehicle_twist"].IsObject()) {
        const auto& vehicle_twist = received_json["vehicle_twist"];
        if (vehicle_twist.HasMember("x") && vehicle_twist["x"].IsNumber() &&
            vehicle_twist.HasMember("y") && vehicle_twist["y"].IsNumber() &&
            vehicle_twist.HasMember("z") && vehicle_twist["z"].IsNumber()) {
            twist[0] = vehicle_twist["x"].GetDouble();
            twist[1] = vehicle_twist["y"].GetDouble();
            twist[2] = vehicle_twist["z"].GetDouble();
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("MosaicClient"), "Invalid or missing fields in 'vehicle_twist'");
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("MosaicClient"), "Missing 'vehicle_twist' in JSON");
    }

    if (received_json.HasMember("siren_active") && received_json["siren_active"].IsBool()) {
        siren_active = received_json["siren_active"].GetBool();
    } else {
        RCLCPP_WARN(rclcpp::get_logger("MosaicClient"), "Missing 'siren_active' in JSON");
    }

    if (received_json.HasMember("light_active") && received_json["light_active"].IsBool()) {
        light_active = received_json["light_active"].GetBool();
    } else {
        RCLCPP_WARN(rclcpp::get_logger("MosaicClient"), "Missing 'light_active' in JSON");
    }

    // Log the extracted information
    RCLCPP_DEBUG(rclcpp::get_logger("MosaicClient"), 
                "Received Vehicle Pose: x=%f, y=%f, z=%f", 
                pose[0], pose[1], pose[2]);
    RCLCPP_DEBUG(rclcpp::get_logger("MosaicClient"), 
                "Received Vehicle Twist: x=%f, y=%f, z=%f", 
                twist[0], twist[1], twist[2]);
    RCLCPP_DEBUG(rclcpp::get_logger("MosaicClient"),
                "Siren Active: %s, Light Active: %s",
                siren_active ? "true" : "false", light_active ? "true" : "false");

    onVehPoseReceived(pose);
    onVehTwistReceived(twist);
    onSirenAndLightStatuReceived(siren_active, light_active);
}


void MosaicClient::received_time(const std::shared_ptr<const std::vector<uint8_t>>& data) {
    const std::vector<uint8_t> vec = *data;
    RCLCPP_DEBUG(rclcpp::get_logger("MosaicClient"), "process_time data received of size: %lu", vec.size());

    std::string json_string(vec.begin(), vec.end());
    RCLCPP_DEBUG(rclcpp::get_logger("MosaicClient"), "process_time data: %s", json_string.c_str());

    // Parse the JSON data
    rapidjson::Document obj;
    std::string timestep_member_name = "timestep";
    obj.Parse(json_string.c_str());
    if (obj.HasParseError()) {
        throw std::runtime_error("Message JSON is misformatted. JSON parsing failed! Please check process_time data: " + json_string);
    }

    unsigned long timestep_received;
    if (obj.HasMember(timestep_member_name.c_str()) && obj.FindMember(timestep_member_name.c_str())->value.IsUint64()) {
        timestep_received = obj[timestep_member_name.c_str()].GetUint64();
    } else {
        throw std::runtime_error("process_time expected unsigned Int nanoseconds as JSON on its time sync udp listener, but was not able to extract member: "
            + timestep_member_name + ". Check if the data is malformed.");
    }

    RCLCPP_DEBUG(rclcpp::get_logger("MosaicClient"), "process_time successfully deserialized unsigned long: %lu", timestep_received);
    onTimeReceived(timestep_received);
    
}
