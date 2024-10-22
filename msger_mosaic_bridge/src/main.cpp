#include "mosaic_adapter.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MosaicAdapter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
