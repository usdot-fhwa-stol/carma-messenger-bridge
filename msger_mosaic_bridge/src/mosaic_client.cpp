#include "mosaic_client.hpp"
#include <rapidjson/document.h>

MosaicClient::MosaicClient() {}

MosaicClient::~MosaicClient() {
    try {
        close();
    } catch (...) {}
}

bool MosaicClient::initialization(const ConnectionConfig& config, boost::system::error_code &ec) {
    bool all_connected = true;

    if (config.enable_registration) {
        all_connected &= conn_manager_.connect(config.ip_address, config.registration_port_remote, config.registration_port_local,
                                            [this](const std::shared_ptr<const std::vector<uint8_t>>& data) { this->received_time(data); }, 
                                            ec, registration_running_);
    }

    if (config.enable_vehicle_status) {
        all_connected &= conn_manager_.connect(config.ip_address, config.vehicle_status_port_remote, config.vehicle_status_port_local,
                                            [this](const std::shared_ptr<const std::vector<uint8_t>>& data) { this->received_vehicle_status(data); }, 
                                            ec, vehicle_status_running_);
    }

    return all_connected;
}

void MosaicClient::close() {
    work_.reset();
    io_->stop();
    io_thread_->join();
    conn_manager_.close("registration", registration_running_);
    conn_manager_.close("vehicle_status", vehicle_status_running_);
}

bool MosaicClient::send_registration_message(const std::shared_ptr<std::vector<uint8_t>>& message) {
    return conn_manager_.send_message("registration", message);
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

    if (received_json.HasMember("vehicle_pose") && received_json["vehicle_pose"].IsObject()) {
        const auto& vehicle_pose = received_json["vehicle_pose"];
        if (vehicle_pose.HasMember("x") && vehicle_pose["x"].IsNumber() &&
            vehicle_pose.HasMember("y") && vehicle_pose["y"].IsNumber() &&
            vehicle_pose.HasMember("z") && vehicle_pose["z"].IsNumber()) {
            pose[0] = vehicle_pose["x"].GetDouble();
            pose[1] = vehicle_pose["y"].GetDouble();
            pose[2] = vehicle_pose["z"].GetDouble();
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

    // Log the extracted information
    RCLCPP_INFO(rclcpp::get_logger("MosaicClient"), 
                "Received Vehicle Pose: x=%f, y=%f, z=%f", 
                pose[0], pose[1], pose[2]);
    RCLCPP_INFO(rclcpp::get_logger("MosaicClient"), 
                "Received Vehicle Twist: x=%f, y=%f, z=%f", 
                twist[0], twist[1], twist[2]);

    onVehStatusReceived(pose, twist);
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
