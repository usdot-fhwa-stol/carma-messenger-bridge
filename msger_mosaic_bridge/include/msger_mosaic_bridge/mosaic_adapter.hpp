#include "mosaic_client.hpp"
#include <rclcpp/rclcpp.hpp>
#include "gps_msgs/msg/gps_fix.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"


class MosaicAdapter : public rclcpp::Node {
public:
    MosaicAdapter();
    virtual ~MosaicAdapter();

private:
    MosaicClient mosaic_client_;
    void initialize();
    void on_message_received(const std::vector<uint8_t>& data);
    void parse_data_to_gps_and_stwist(const std::vector<uint8_t>& data, gps_msgs::msg::GPSFix& gps, geometry_msgs::msg::TwistStamped& twist);
    rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gps_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
    boost::system::error_code client_error_;
};

