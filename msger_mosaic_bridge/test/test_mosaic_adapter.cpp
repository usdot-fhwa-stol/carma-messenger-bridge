#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "mosaic_adapter.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include <boost/asio.hpp>
#include <thread>
#include <future>
#include <iostream>

// Fixture for MosaicAdapter tests
class MosaicAdapterTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize ROS2 and create the adapter instance
        mosaic_adapter_node_ = std::make_shared<MosaicAdapter>();
    }

    std::shared_ptr<MosaicAdapter> mosaic_adapter_node_;
};

// Test GPSFix publication when onVehPoseReceived is triggered
TEST_F(MosaicAdapterTest, TestGPSFixPublication) {
    // Create a subscriber to the "vehicle_pose" topic
    auto gps_subscriber = mosaic_adapter_node_->create_subscription<gps_msgs::msg::GPSFix>(
        "vehicle_pose", 10, [this](const gps_msgs::msg::GPSFix::SharedPtr msg) {
            // Verify the content of the published message
            EXPECT_DOUBLE_EQ(msg->latitude, 1.23); 
            EXPECT_DOUBLE_EQ(msg->longitude, 4.56);
            EXPECT_DOUBLE_EQ(msg->altitude, 7.89);
        });

    // Trigger the onVehPoseReceived signal
    std::array<double, 3> test_pose = {1.23, 4.56, 7.89};
    mosaic_adapter_node_->mosaic_client_.onVehPoseReceived(test_pose);

    // Wait for a message to be received
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(mosaic_adapter_node_);
}

// Test TwistStamped publication when onVehTwistReceived is triggered
TEST_F(MosaicAdapterTest, TestTwistStampedPublication) {
    // Create a subscriber to the "velocity" topic
    auto twist_subscriber = mosaic_adapter_node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        "velocity", 10, [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
            // Verify the content of the published message
            EXPECT_DOUBLE_EQ(msg->twist.linear.x, 0.1);
            EXPECT_DOUBLE_EQ(msg->twist.linear.y, 0.2);
            EXPECT_DOUBLE_EQ(msg->twist.linear.z, 0.3);
        });

    // Trigger the onVehTwistReceived signal
    std::array<double, 3> test_twist = {0.1, 0.2, 0.3};
    mosaic_adapter_node_->mosaic_client_.onVehTwistReceived(test_twist);

    // Wait for a message to be received
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(mosaic_adapter_node_);
}

// Test Clock publication when onTimeReceived is triggered
TEST_F(MosaicAdapterTest, TestClockPublication) {
    // Create a subscriber to the "/sim_clock" topic
    auto clock_subscriber = mosaic_adapter_node_->create_subscription<rosgraph_msgs::msg::Clock>(
        "/sim_clock", 10, [this](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
            // Verify the content of the published message
            EXPECT_EQ(msg->clock.sec, 1);
            EXPECT_EQ(msg->clock.nanosec, 500000000);
        });

    // Trigger the onTimeReceived signal
    unsigned long test_time = 1500000000;
    mosaic_adapter_node_->mosaic_client_.onTimeReceived(test_time);

    // Wait for a message to be received
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(mosaic_adapter_node_);
}

TEST_F(MosaicAdapterTest, TestServerReceivesHandshakeMessage) {
    // Setup a UDP server using Boost Asio to listen for the handshake message

    // Extract the relevant configuration values from the MosaicAdapter
    const std::string server_ip = "127.0.0.1"; // Typically, localhost is used for tests.
    const int server_port = 6000; // Example port - replace with the actual value from your config if different.

    boost::asio::io_context io_context;
    boost::asio::ip::udp::socket socket(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::make_address(server_ip), server_port));
    
    // Prepare a buffer for receiving the handshake message
    std::array<char, 1024> recv_buffer;
    boost::asio::ip::udp::endpoint client_endpoint;

    RCLCPP_INFO(rclcpp::get_logger("MosaicAdapterTest"), "Server waiting for message on IP: %s, Port: %d", server_ip.c_str(), server_port);

    // Set up a flag to indicate if the timeout has occurred
    bool timeout_occurred = false;
    bool message_received = false;

    // Set up a timer to avoid blocking indefinitely
    boost::asio::steady_timer timeout_timer(io_context);
    timeout_timer.expires_after(std::chrono::seconds(2));
    timeout_timer.async_wait([&](const boost::system::error_code& ec) {
        if (!ec) {
            RCLCPP_WARN(rclcpp::get_logger("MosaicAdapterTest"), "Server receive timeout reached.");
            timeout_occurred = true;
        }
    });

    // Start the asynchronous receive
    socket.async_receive_from(
        boost::asio::buffer(recv_buffer), client_endpoint,
        [&](const boost::system::error_code& ec, std::size_t len) {
            if (!ec && len > 0) {
                // Extract the received message as a string
                std::string received_msg(recv_buffer.data(), len);
                RCLCPP_INFO(rclcpp::get_logger("MosaicAdapterTest"), "Server received handshake message: %s", received_msg.c_str());

                // Verify that the message is received correctly (adjust checks based on expected content)
                EXPECT_TRUE(received_msg.find("sumoVehicleRole") != std::string::npos);
                EXPECT_TRUE(received_msg.find("rxIpAddress") != std::string::npos);
                EXPECT_TRUE(received_msg.find("rxVehicleStatusPort") != std::string::npos);

                message_received = true;
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("MosaicAdapterTest"), "Error receiving from UDP server: %s", ec.message().c_str());
            }
        });

    // Call the mosaic adapter initialize after the server is ready
    mosaic_adapter_node_->initialize();

    // Run the IO context and wait for either the message or the timeout
    while (!message_received && !timeout_occurred) {
        io_context.run_one();
    }

    // Check if the message was received within the timeout
    EXPECT_TRUE(message_received);
}

int main(int argc, char **argv) {
    // Initialize Google Test and ROS2
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);

    int result = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return result;
}
