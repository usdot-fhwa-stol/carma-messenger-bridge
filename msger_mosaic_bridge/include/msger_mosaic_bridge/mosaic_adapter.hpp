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
#include "carma_msgs/srv/set_traffic_event.hpp"
#include <rapidjson/document.h>
#include "std_srvs/srv/trigger.hpp"


// The MosaicAdapter class adapts the MosaicClient for use within a ROS2 environment, facilitating the communication between
// the ambassador and ROS2 topics by publishing the received network data as ROS2 messages.
class MosaicAdapter : public rclcpp::Node {
public:
    MosaicAdapter();
    ~MosaicAdapter() { shutdown(); };
    void sendHandshake();
    MosaicClient mosaic_client_;
private:

    void shutdown();


    // Callback method that is invoked when vehicle status data is received from the network.
    // It translates the received data into ROS2 GPS and Twist messages and publishes them on the appropriate topics.
    void on_vehicle_twist_received_handler(const std::array<double, 3>& twist);
    void on_vehicle_pose_received_handler(const std::array<double, 3>& pose);

    // Callback method that handles time synchronization messages received from the network. It publishes the time data
    // as ROS2 Clock messages to synchronize components within the ROS2 environment.
    void on_time_received_handler(unsigned long timestamp);

    void on_siren_and_light_status_recieved_handler(bool siren_active, bool light_active);

    void on_traffic_event_received_handler(float up_track, float down_track, float minimum_gap, float advisory_speed);
    
    // Utility method to compose a JSON-formatted handshake message that is used to establish initial communication with the
    // network server. This message includes necessary configuration details such as role ID and IP addresses.
    std::string compose_handshake_msg(const std::string& role_id, int veh_status_port, int traffic_event_port, int time_port, const std::string& ip);
    
    // Sends the handshake message to the network server to initiate communication. This is crucial for setting up the network
    // configuration and ensuring that the server is aware of the client's presence and ready to exchange data.
    void broadcast_handshake_msg(const std::string& msg_string);

    rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr time_pub_;
    rclcpp::Client<carma_msgs::srv::SetTrafficEvent>::SharedPtr start_broadcasting_traffic_event_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_broadcasting_traffic_event_client_;

    boost::system::error_code client_error_;
    ConnectionConfig config_;
};

