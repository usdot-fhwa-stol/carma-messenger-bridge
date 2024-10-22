#include "mosaic_client.hpp"
#include <rclcpp/rclcpp.hpp>
#include "gps_msgs/msg/gps_fix.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rosgraph_msgs/msg/clock.hpp"


class MosaicAdapter : public rclcpp::Node {
public:
    MosaicAdapter();
    virtual ~MosaicAdapter();

private:
    MosaicClient mosaic_client_;
    void initialize();

    void on_vehicle_status_received_handler(const std::array<double, 3>& pose, const std::array<double, 3>& twist);
    void on_time_received_handler(unsigned long timestamp);
    
    rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gps_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr time_pub_;

    boost::system::error_code client_error_;
    ConnectionConfig config_;
};

