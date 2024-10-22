#include "mosaic_client.hpp"
#include <rapidjson/document.h>

MosaicClient::MosaicClient() {}

MosaicClient::~MosaicClient() {
    try {
        close();
    } catch (...) {}
}

void MosaicClient::initialization(const ConnectionConfig& config, boost::system::error_code &ec) {
    // if (config.enable_v2x) {
    //     conn_manager_.connect(config.ip_address, config.v2x_port_remote, config.v2x_port_local,
    //         [this](const std::shared_ptr<const std::vector<uint8_t>>& data) {
    //             this->received_v2x(data);
    //         }, ec, v2x_running_);
    // }

    if (config.enable_registration) {
        conn_manager_.connect(config.ip_address, config.registration_port_remote, config.registration_port_local,
            [this](const std::shared_ptr<const std::vector<uint8_t>>& data) {
                this->received_time(data);
            }, ec, registration_running_);
    }

    if (config.enable_vehicle_status) {
        conn_manager_.connect(config.ip_address, config.vehicle_status_port_remote, config.vehicle_status_port_local,
            [this](const std::shared_ptr<const std::vector<uint8_t>>& data) {
                this->received_vehicle_status(data);
            }, ec, vehicle_status_running_);
    }
}

void MosaicClient::close() {
    work_.reset();
    io_->stop();
    io_thread_->join();
    // conn_manager_.close("v2x", v2x_running_);
    conn_manager_.close("registration", registration_running_);
    conn_manager_.close("vehicle_status", vehicle_status_running_);
}

// bool MosaicClient::send_v2x_message(const std::shared_ptr<std::vector<uint8_t>>& message) {
//     if (!v2x_running_) return false;
//     return conn_manager_.send_message("v2x", message);
// }

bool MosaicClient::send_registration_message(const std::shared_ptr<std::vector<uint8_t>>& message) {
    return conn_manager_.send_message("registration", message);
}

// void MosaicClient::received_v2x(const std::shared_ptr<const std::vector<uint8_t>>& data) {
//     RCLCPP_DEBUG(rclcpp::get_logger("MosaicClient"), "Received V2X data");

//         auto & entry = *data;
//     // Valid message should begin with 2 bytes message ID and 1 byte length.
//     for (size_t i = 0; i < entry.size() - 3; i++) { // leave 3 bytes after (for lsb of id, length byte 1, and either message body or length byte 2)
//         // Generate the 16-bit message id from two bytes, skip if it isn't a valid one
//         uint16_t msg_id = (static_cast<uint16_t>(entry[i]) << 8) | static_cast<uint16_t>(entry[i + 1]);

//         // Parse the length, check it doesn't run over
//         size_t len = 0;
//         size_t len_byte_1 = entry[i + 2];
//         int len_bytes = 0;
//         // length < 128 encoded by single byte with msb set to 0
//         if ((len_byte_1 & 0x80 ) == 0x00) {
//             len = static_cast<size_t>(len_byte_1);
//             len_bytes = 1;
//             // check for 0 length
//             if (len_byte_1 == 0x00) { continue; }
//         }
//             // length < 16384 encoded by 14 bits in 2 bytes (10xxxxxx xxxxxxxx)
//         else if ((len_byte_1 & 0x40) == 0x00) { //we know msb = 1, check that next bit is 0
//             if (i + 3 < entry.size()) {
//                 size_t len_byte_2 = entry[i + 3];
//                 len = ((len_byte_1 & 0x3f) << 8) | len_byte_2;
//                 len_bytes = 2;
//             }
//         }
//         else {
//             // TODO lengths greater than 16383 (0x3FFF) are encoded by splitting up the message into discrete chunks, each with its own length
//             // marker. It doesn't look like we'll be receiving anything that long
//             std::cerr << "NS3Client::process() : received a message with length field longer than 16383." << std::endl;
//             continue;
//         }
//         if (len == -1) { continue; }
//         // If the length makes sense bsmPub(fits in the buffer), copy out the message bytes and pass to the Application class
//         if ((i + 1 + len + len_bytes) < entry.size()) {
//             // bool found_valid_msg = true;
//             size_t start_index = i;
//             size_t end_index = i + 2 + len + len_bytes; // includes 2 msgID bytes before message body
//             //this constructor has range [first, last) hence the + 1
//             std::vector<uint8_t> msg_vec(entry.begin() + start_index, entry.begin() + end_index);
//             onMessageReceived(msg_vec, msg_id);
//             break;
//         }
//     }
// }

void MosaicClient::received_vehicle_status(const std::shared_ptr<const std::vector<uint8_t>>& data) {
    RCLCPP_DEBUG(rclcpp::get_logger("MosaicClient"), "Received vehicle status data");
    // onVehStatusReceived();
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

    // Call your method to handle the received time
    onTimeReceived(timestep_received);
    
}
