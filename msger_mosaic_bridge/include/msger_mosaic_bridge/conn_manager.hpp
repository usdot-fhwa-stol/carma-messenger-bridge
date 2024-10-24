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


#ifndef CONN_MANAGER_HPP
#define CONN_MANAGER_HPP

#include <unordered_map>
#include <memory>
#include <string>
#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <vector>
#include "udp_listener.hpp"

class ConnectionManager {
public:
    using MessageHandler = std::function<void(const std::shared_ptr<const std::vector<uint8_t>>&)>;

    /**
     * Establishes a UDP connection.
     * @param connectionType A string identifier for the connection type (e.g., "vehicle_status").
     * @param remote_address The IP address of the remote endpoint.
     * @param remote_port The port number of the remote endpoint.
     * @param local_port The port number on the local machine to bind for this connection.
     * @param handler The function to call when messages are received.
     * @param ec A reference to a variable to store the error code if the connection fails.
     * @param is_running A reference to a boolean flag indicating if the connection is active.
     * @return true if the connection was established successfully, otherwise false.
     */
    bool connect(const std::string &connectionType,
                 const std::string &remote_address,
                 unsigned short remote_port,
                 unsigned short local_port,
                 MessageHandler handler,
                 boost::system::error_code &ec,
                 bool &is_running);

    /**
     * Sends a message via a specific connection type.
     * @param connection_type The type of connection over which the message will be sent.
     * @param message The message data to be sent, encapsulated in a shared pointer to a vector of bytes.
     * @return true if the message was sent successfully, false otherwise.
     */
    bool send_message(const std::string &connection_type, const std::shared_ptr<std::vector<uint8_t>>& message);
    boost::signals2::signal<void(const boost::system::error_code&)> onError;

    /**
     * Closes a specified connection.
     * @param connection_type The identifier of the connection to close.
     * @param is_running Reference to a boolean that indicates the connection's active state, set to false on close.
     */
    void close(const std::string &connection_type, bool &is_running);

private:
    std::unordered_map<std::string, std::unique_ptr<cav::UDPListener>> listeners_;
    std::unordered_map<std::string, std::unique_ptr<boost::asio::ip::udp::socket>> sockets_;
    std::unordered_map<std::string, boost::asio::ip::udp::endpoint> remote_endpoints_;

    std::unique_ptr<boost::asio::io_context> io_;
    std::unique_ptr<boost::asio::io_context::strand> output_strand_;

};

#endif // CONN_MANAGER_HPP
