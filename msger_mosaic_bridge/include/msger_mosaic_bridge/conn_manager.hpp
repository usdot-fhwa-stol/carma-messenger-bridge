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

    bool connect(const std::string &remote_address,
                 unsigned short remote_port,
                 unsigned short local_port,
                 MessageHandler handler,
                 boost::system::error_code &ec,
                 bool &is_running);

    bool send_message(const std::string &connection_type, const std::shared_ptr<std::vector<uint8_t>>& message);

    void close(const std::string &connection_type, bool &is_running);

private:
    std::unordered_map<std::string, std::unique_ptr<cav::UDPListener>> listeners_;
    std::unordered_map<std::string, std::unique_ptr<boost::asio::ip::udp::socket>> sockets_;
    std::unordered_map<std::string, boost::asio::ip::udp::endpoint> remote_endpoints_;

    std::unique_ptr<boost::asio::io_context> io_;
    std::unique_ptr<boost::asio::io_context::strand> output_strand_;
};

#endif // CONN_MANAGER_HPP
