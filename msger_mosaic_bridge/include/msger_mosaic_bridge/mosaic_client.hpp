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


#ifndef MOSAIC_CLIENT_HPP
#define MOSAIC_CLIENT_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "conn_manager.hpp"

struct ConnectionConfig {
    std::string role_id = "";
    std::string vehicle_id = "";
    bool enable_registration = false;
    bool enable_vehicle_status = false;

    std::string ip_address = "192.168.1.100"; 

    unsigned short registration_port_remote = 6000;
    unsigned short registration_port_local = 4001;

    unsigned short vehicle_status_port_remote = 7000;
    unsigned short vehicle_status_port_local = 4002;
};

class MosaicClient {
public:
    MosaicClient();
    ~MosaicClient();
    /**
     * Initializes connections based on the provided configuration.
     * @param config Connection configuration settings.
     * @param ec Output parameter to capture any error that occurs during initialization.
     * @return Returns true if all connections are successfully initialized, false otherwise.
     */
    bool initialization(const ConnectionConfig& config, boost::system::error_code &ec);

    /**
     * Sends a registration message.
     * @param message The message to be sent, encapsulated in a shared pointer to a vector of bytes.
     * @return Returns true if the message was sent successfully, false otherwise.
     */
    bool send_registration_message(const std::shared_ptr<std::vector<uint8_t>>& message);

    void close();

    boost::signals2::signal<void(unsigned long)> onTimeReceived;
    boost::signals2::signal<void(const std::array<double, 3>& , const std::array<double, 3>& )> onVehStatusReceived;


private:
    void received_v2x(const std::shared_ptr<const std::vector<uint8_t>>& data);
    void received_vehicle_status(const std::shared_ptr<const std::vector<uint8_t>>& data);
    void received_time(const std::shared_ptr<const std::vector<uint8_t>>& data);

    ConnectionManager conn_manager_;
    bool vehicle_status_running_ = false;
    bool registration_running_ = false;
    std::unique_ptr<boost::asio::io_context> io_;
    std::unique_ptr<std::thread> io_thread_;
    std::unique_ptr<boost::asio::io_context::work> work_;
    boost::system::error_code conn_manager_error_;
};

#endif // MOSAIC_CLIENT_HPP
