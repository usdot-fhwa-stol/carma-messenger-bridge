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

    std::string messenger_ip_address = "127.0.0.1";
    std::string cdasim_ip_address = "127.0.0.1"; 

    unsigned short siren_and_light_status_port_remote = 6000;
    unsigned short siren_and_light_status_port_local = 6000;

    unsigned short registration_port_remote = 6000;
    unsigned short registration_port_local = 4001;

    unsigned short vehicle_status_port_remote = 7000;
    unsigned short vehicle_status_port_local = 4002;
};

enum SirensAndLightsStatus : uint8_t
  {
    SIRENS_AND_LIGHTS_INACTIVE = 49,  // Value of char '1'
    ONLY_SIRENS_ACTIVE = 50,          // Value of char '2'
    ONLY_LIGHTS_ACTIVE = 51,          // Value of char '3'
    SIRENS_AND_LIGHTS_ACTIVE = 52     // Value of char '4'
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
    bool initialize(const ConnectionConfig& config, boost::system::error_code &ec);

    /**
     * Sends a registration message.
     * @param message The message to be sent, encapsulated in a shared pointer to a vector of bytes.
     * @return Returns true if the message was sent successfully, false otherwise.
     */
    bool send_registration_message(const std::shared_ptr<std::vector<uint8_t>>& message);

    bool send_siren_and_light_message(const std::shared_ptr<std::vector<uint8_t>>& message);

    void close();

    boost::signals2::signal<void(unsigned long)> onTimeReceived;
    boost::signals2::signal<void(const std::array<double, 3>&)> onVehPoseReceived;
    boost::signals2::signal<void(const std::array<double, 3>&)> onVehTwistReceived;
    boost::signals2::signal<void(bool siren_active, bool light_active)> onSirenAndLightStatuReceived;


private:
    void received_vehicle_status(const std::shared_ptr<const std::vector<uint8_t>>& data);
    void received_time(const std::shared_ptr<const std::vector<uint8_t>>& data);

    ConnectionManager conn_manager_;
    bool vehicle_status_running_ = false;
    bool registration_running_ = false;
    bool siren_and_light_running_ = false;
    std::unique_ptr<boost::asio::io_context> io_;
    std::unique_ptr<std::thread> io_thread_;
    std::unique_ptr<boost::asio::io_context::work> work_;
    boost::system::error_code conn_manager_error_;
};

#endif // MOSAIC_CLIENT_HPP
