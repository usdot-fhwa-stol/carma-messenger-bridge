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


#include "conn_manager.hpp"
#include "udp_listener.hpp"

bool ConnectionManager::connect(const std::string &connectionType,
                const std::string &remote_address,
                unsigned short remote_port,
                unsigned short local_port,
                MessageHandler handler,
                boost::system::error_code &ec,
                bool &is_running) 
{
    if (is_running) {
        RCLCPP_WARN(rclcpp::get_logger("ConnectionManager"), "Connection is already running");
        return false;
    }

    // Log the input parameters
    RCLCPP_INFO(rclcpp::get_logger("ConnectionManager"), "Attempting to connect: %s, %s, %u", 
                connectionType.c_str(), remote_address.c_str(), remote_port);

    if (!io_) {
        RCLCPP_INFO(rclcpp::get_logger("ConnectionManager"), "Initializing io_context");
        io_.reset(new boost::asio::io_context());
        output_strand_.reset(new boost::asio::io_context::strand(*io_));

        // Create a work guard to keep the io_context running
        work_guard_ = std::make_unique<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>>(io_->get_executor());

        io_thread_ = std::make_shared<std::thread>([this]() {
            RCLCPP_INFO(rclcpp::get_logger("ConnectionManager"), "Starting io_context run loop in background thread.");
            try {
                io_->run();  // Run the io_context until explicitly stopped
            } catch (const boost::system::system_error& e) {
                RCLCPP_ERROR(rclcpp::get_logger("ConnectionManager"), "Exception in io_context run loop (code: %d, message: %s): %s", 
                            e.code().value(), e.code().message().c_str(), e.what());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("ConnectionManager"), "General exception in io_context run loop: %s", e.what());
            } catch (...) {
                RCLCPP_ERROR(rclcpp::get_logger("ConnectionManager"), "Unknown exception in io_context run loop.");
            }
        });
        
    }

    boost::asio::ip::udp::endpoint remote_udp_ep;
    remote_udp_ep = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(remote_address, ec), remote_port);
    if (ec) {
        return false;
    }

    try {
        if (handler) {
            std::unique_ptr<cav::UDPListener> listener;
            listener.reset(new cav::UDPListener(*io_, local_port));
            listener->onReceive.connect(handler);
            listener->start();
            listeners_[connectionType] = std::move(listener);
        }

        // Create and configure the socket
        std::unique_ptr<boost::asio::ip::udp::socket> socket;
        socket.reset(new boost::asio::ip::udp::socket(*io_, remote_udp_ep.protocol()));
        sockets_[connectionType] = std::move(socket);
        remote_endpoints_[connectionType] = remote_udp_ep;

        is_running = true;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("ConnectionManager"), "Error setting up connection: %s", e.what());
        ec = boost::asio::error::host_not_found;  // Example error code
        return false;
    }

    return true;
}

bool ConnectionManager::send_message(const std::string &connection_type, const std::shared_ptr<std::vector<uint8_t>>& message) {
    
    if (sockets_.find(connection_type) == sockets_.end()) {
        RCLCPP_ERROR(rclcpp::get_logger("ConnectionManager"), "Socket not found for connection type: %s", connection_type.c_str());
        return false;
    }

    try {
        if (!output_strand_ || !io_) {
            RCLCPP_ERROR(rclcpp::get_logger("ConnectionManager"), "Strand or IO context not properly initialized");
            return false;
        }

        auto socket = sockets_[connection_type].get();
        auto endpoint = remote_endpoints_[connection_type];

        // Log the local endpoint from which the message will be sent
        if (socket->is_open()) {
            boost::system::error_code ec;
            auto local_endpoint = socket->local_endpoint(ec);
            if (!ec) {
                RCLCPP_INFO(rclcpp::get_logger("ConnectionManager"),
                            "Sending message from local endpoint %s:%u through socket for connection type: %s, to remote endpoint IP: %s, Port: %u",
                            local_endpoint.address().to_string().c_str(),
                            local_endpoint.port(),
                            connection_type.c_str(),
                            endpoint.address().to_string().c_str(),
                            endpoint.port());
                // socket->send_to(boost::asio::buffer(*message), endpoint);
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("ConnectionManager"),
                            "Failed to retrieve local endpoint for connection type: %s, Error: %s",
                            connection_type.c_str(), ec.message().c_str());
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("ConnectionManager"), "Socket is not open for connection type: %s", connection_type.c_str());
            return false;
        }

        output_strand_->post([this, socket, endpoint, message]() {
            try {
                if (socket && socket->is_open()) {
                    onSent(socket->send_to(boost::asio::buffer(*message), endpoint));
                }
            } catch (const boost::system::system_error& error_code) {
                onError(error_code.code());
            } catch (...) {
                onError(boost::asio::error::fault);
            }
        });

        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ConnectionManager"), "Exception posting message: %s", e.what());
        return false;
    }
}

void ConnectionManager::close(const std::string &connection_type, bool &is_running) {
    if (!is_running) {
        RCLCPP_WARN(rclcpp::get_logger("ConnectionManager"), "%s connection is not running", connection_type.c_str());
        return;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("ConnectionManager"), "Trying to stop %s listeners", connection_type.c_str());

    // Stop and remove the listener associated with the connection type, if it exists
    auto listener_it = listeners_.find(connection_type);
    if (listener_it != listeners_.end()) {
        listener_it->second->stop(); // Stops the UDPListener instance, not the socket
        RCLCPP_DEBUG(rclcpp::get_logger("ConnectionManager"), "%s listener is stopped", connection_type.c_str());
        listeners_.erase(listener_it);
    }

    // Close and remove the socket associated with the connection type
    auto socket_it = sockets_.find(connection_type);
    if (socket_it != sockets_.end()) {
        if (socket_it->second->is_open()) {
            // Cancel all asynchronous operations on the socket
            boost::system::error_code cancel_ec;
            socket_it->second->cancel(cancel_ec);
            if (cancel_ec) {
                RCLCPP_ERROR(rclcpp::get_logger("ConnectionManager"), "Error canceling socket operations for %s: %s", connection_type.c_str(), cancel_ec.message().c_str());
            }

            // Close the socket
            boost::system::error_code ec;
            socket_it->second->close(ec);
            if (ec) {
                RCLCPP_ERROR(rclcpp::get_logger("ConnectionManager"), "Error closing socket for connection %s: %s", connection_type.c_str(), ec.message().c_str());
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("ConnectionManager"), "Socket for connection %s was already closed", connection_type.c_str());
        }
        sockets_.erase(socket_it);
    } else {
        RCLCPP_WARN(rclcpp::get_logger("ConnectionManager"), "No socket found for connection type: %s", connection_type.c_str());
    }

    // Remove the remote endpoint associated with the connection type
    remote_endpoints_.erase(connection_type);

    // Release the work guard to allow io_context to stop
    if (work_guard_) {
        work_guard_.reset(); // Releases the work guard and lets io_context stop
    }

    // Stop the io_context gracefully if it's still running
    if (io_) {
        boost::system::error_code ec;
        io_->stop();
    }

    // Optionally, join the thread if it's still running
    if (io_thread_ && io_thread_->joinable()) {
        io_thread_->join();
    }

    // Mark connection as not running
    is_running = false;
}

