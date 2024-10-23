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

    bool initialization(const ConnectionConfig& config, boost::system::error_code &ec);

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
};

#endif // MOSAIC_CLIENT_HPP
