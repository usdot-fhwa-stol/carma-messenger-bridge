#include "conn_manager.hpp"
#include "udp_listener.hpp"

bool ConnectionManager::connect(const std::string &remote_address,
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

    if (!io_) {
        // io_ = std::make_unique<boost::asio::io_context>();
        io_.reset(new boost::asio::io_context());
        // output_strand_ = std::make_unique<boost::asio::strand<boost::asio::io_context::executor_type>>(io_->get_executor());
        output_strand_.reset(new boost::asio::io_context::strand(*io_));
    }

    boost::asio::ip::udp::endpoint remote_udp_ep(boost::asio::ip::address::from_string(remote_address, ec), remote_port);
    if (ec) {
        return false;
    }

    try {
        std::unique_ptr<cav::UDPListener> listener;
        listener.reset(new cav::UDPListener(*io_, local_port));
        listener->onReceive.connect(handler);
        listener->start();

        listeners_[remote_address] = std::move(listener);

        auto socket = std::make_unique<boost::asio::ip::udp::socket>(*io_, remote_udp_ep.protocol());
        sockets_[remote_address] = std::move(socket);
        remote_endpoints_[remote_address] = remote_udp_ep;

        is_running = true;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("ConnectionManager"), "Error setting up connection: %s", e.what());
        ec = boost::asio::error::host_not_found;  // Example error code
        return false;
    }

    return true;
}

bool ConnectionManager::send_message(const std::string &connection_type, const std::shared_ptr<std::vector<uint8_t>>& message) {
    // Check if the connection is running
    if (sockets_.find(connection_type) == sockets_.end()) {
        RCLCPP_ERROR(rclcpp::get_logger("ConnectionManager"), "Socket not found for connection type: %s", connection_type.c_str());
        return false;
    }

    try {
        // Ensure `output_strand_` and `io_context` are initialized before posting
        if (!output_strand_ || !io_) {
            RCLCPP_ERROR(rclcpp::get_logger("ConnectionManager"), "Strand or IO context not properly initialized");
            return false;
        }

        output_strand_->post([this, connection_type, message]() {
            try {
                auto socket = sockets_[connection_type].get();
                auto endpoint = remote_endpoints_[connection_type];

                if (socket) {
                    socket->send_to(boost::asio::buffer(*message), endpoint);
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("ConnectionManager"), "Invalid socket for connection type: %s", connection_type.c_str());
                }
            } catch (const boost::system::system_error& error_code) {
                RCLCPP_ERROR(rclcpp::get_logger("ConnectionManager"), "Failed to send message: %s", error_code.what());
            } catch (...) {
                RCLCPP_ERROR(rclcpp::get_logger("ConnectionManager"), "Unknown error while sending message");
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

    auto it = listeners_.find(connection_type);
    if (it != listeners_.end()) {
        it->second->stop();
        listeners_.erase(it);
        sockets_.erase(connection_type);
        remote_endpoints_.erase(connection_type);
        is_running = false;
    }
}
