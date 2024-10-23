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
#include <rclcpp/rclcpp.hpp>
#include "gps_msgs/msg/gps_fix.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include <rapidjson/document.h>

// The MosaicAdapter class adapts the MosaicClient for use within a ROS2 environment, facilitating the communication between
// the ambassador and ROS2 topics by publishing the received network data as ROS2 messages.
class MosaicAdapter : public rclcpp::Node {
public:
    MosaicAdapter();
    ~MosaicAdapter() { shutdown(); };
private:

    void shutdown();

    void initialize();

    // Callback method that is invoked when vehicle status data is received from the network.
    // It translates the received data into ROS2 GPS and Twist messages and publishes them on the appropriate topics.
    void on_vehicle_status_received_handler(const std::array<double, 3>& pose, const std::array<double, 3>& twist);

    // Callback method that handles time synchronization messages received from the network. It publishes the time data
    // as ROS2 Clock messages to synchronize components within the ROS2 environment.
    void on_time_received_handler(unsigned long timestamp);
    
    // Utility method to compose a JSON-formatted handshake message that is used to establish initial communication with the
    // network server. This message includes necessary configuration details such as role ID and IP addresses.
    std::string compose_handshake_msg(const std::string& role_id, int veh_status_port, int time_port, const std::string& ip);
    
    // Sends the handshake message to the network server to initiate communication. This is crucial for setting up the network
    // configuration and ensuring that the server is aware of the client's presence and ready to exchange data.
    void broadcast_handshake_msg(const std::string& msg_string);

    rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gps_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr time_pub_;

    boost::system::error_code client_error_;
    ConnectionConfig config_;
    MosaicClient mosaic_client_;
};

