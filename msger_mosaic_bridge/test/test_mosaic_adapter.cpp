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

TEST_F(MosaicAdapterTest, TestMsgerReceiveSirenAndLightStatus) {
    const std::string server_ip = "127.0.0.1";
    const int server_port = 8001;

    boost::asio::io_context io_context;
    boost::asio::ip::udp::socket socket(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::make_address(server_ip), server_port));

    // Prepare a buffer for receiving the status message
    std::array<char, 1024> recv_buffer;
    boost::asio::ip::udp::endpoint client_endpoint;

    RCLCPP_INFO(rclcpp::get_logger("MosaicAdapterTest"), "Server waiting for siren and light status message on IP: %s, Port: %d", server_ip.c_str(), server_port);

    // Flags to indicate the status of the message receipt and timeout
    bool timeout_occurred = false;
    bool message_received = false;

    // Set up a timer to avoid blocking indefinitely
    boost::asio::steady_timer timeout_timer(io_context);
    timeout_timer.expires_after(std::chrono::seconds(2)); // Adjust timeout duration if needed
    timeout_timer.async_wait([&](const boost::system::error_code& ec) {
        if (!ec) {
            RCLCPP_WARN(rclcpp::get_logger("MosaicAdapterTest"), "Server receive timeout reached.");
            timeout_occurred = true;
            boost::system::error_code cancel_ec;
            socket.cancel(cancel_ec); // Cancel pending receive operation to prevent further waiting
            if (cancel_ec) {
                RCLCPP_ERROR(rclcpp::get_logger("MosaicAdapterTest"), "Failed to cancel socket operations: %s", cancel_ec.message().c_str());
            }
        }
    });

    // Start the asynchronous receive operation
    socket.async_receive_from(
        boost::asio::buffer(recv_buffer), client_endpoint,
        [&](const boost::system::error_code& ec, std::size_t len) {
            if (ec == boost::asio::error::operation_aborted) {
                RCLCPP_WARN(rclcpp::get_logger("MosaicAdapterTest"), "Receive operation was canceled due to timeout or closure.");
                return;
            }

            if (!socket.is_open()) {
                RCLCPP_ERROR(rclcpp::get_logger("MosaicAdapterTest"), "Socket is not open during receive operation.");
                return;
            }

            if (!ec && len > 0) {
                // Extract the received status code
                uint8_t received_status_code = static_cast<uint8_t>(recv_buffer[0]);
                RCLCPP_INFO(rclcpp::get_logger("MosaicAdapterTest"), "Server received siren and light status message: %u", received_status_code);

                // Determine the expected status code based on siren and light states
                bool siren_active = true;
                bool light_active = true;
                uint8_t expected_status_code = 0;
                if (!siren_active && !light_active) {
                    expected_status_code = SIRENS_AND_LIGHTS_INACTIVE;
                } else if (siren_active && !light_active) {
                    expected_status_code = ONLY_SIRENS_ACTIVE;
                } else if (!siren_active && light_active) {
                    expected_status_code = ONLY_LIGHTS_ACTIVE;
                } else if (siren_active && light_active) {
                    expected_status_code = SIRENS_AND_LIGHTS_ACTIVE;
                }

                // Verify that the received message matches the expected status code
                EXPECT_EQ(received_status_code, expected_status_code);

                message_received = true;
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("MosaicAdapterTest"), "Error receiving from UDP server: %s", ec.message().c_str());
            }
        });

    // Trigger the sending of the siren and light status message
    bool siren_active = true;
    bool light_active = true;
    mosaic_adapter_node_->mosaic_client_.onSirenAndLightStatuReceived(siren_active, light_active);

    // Run the IO context and wait for either the message or the timeout
    while (!message_received && !timeout_occurred) {
        io_context.run_one(); // Process one event at a time
    }

    // Before exiting, close the socket properly
    if (socket.is_open()) {
        boost::system::error_code ec;
        socket.close(ec);
        if (ec) {
            RCLCPP_ERROR(rclcpp::get_logger("MosaicAdapterTest"), "Error closing socket: %s", ec.message().c_str());
        }
    }

    // Check if the message was received within the timeout period
    EXPECT_TRUE(message_received);
}

TEST_F(MosaicAdapterTest, TestMosaicReceivesHandshakeMessage) {

    // Extract the relevant configuration values from the MosaicAdapter
    const std::string server_ip = "127.0.0.1"; 
    const int server_port = 6001;

    // Create the io_context and a UDP socket bound to the server IP and port
    boost::asio::io_context io_context;
    boost::asio::ip::udp::socket socket(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::make_address(server_ip), server_port));
    
    // Prepare a buffer for receiving the handshake message
    std::array<char, 1024> recv_buffer;
    boost::asio::ip::udp::endpoint client_endpoint;

    RCLCPP_INFO(rclcpp::get_logger("MosaicAdapterTest"), "Server waiting for message on IP: %s, Port: %d", server_ip.c_str(), server_port);

    // Flags to indicate the status of the message receipt and timeout
    bool timeout_occurred = false;
    bool message_received = false;

    // Set up a timer to avoid blocking indefinitely
    boost::asio::steady_timer timeout_timer(io_context);
    timeout_timer.expires_after(std::chrono::seconds(2)); // Adjust timeout duration if needed
    timeout_timer.async_wait([&](const boost::system::error_code& ec) {
        if (!ec) {
            RCLCPP_WARN(rclcpp::get_logger("MosaicAdapterTest"), "Server receive timeout reached.");
            timeout_occurred = true;
            boost::system::error_code cancel_ec;
            socket.cancel(cancel_ec); // Cancel pending receive operation to prevent further waiting
            if (cancel_ec) {
                RCLCPP_ERROR(rclcpp::get_logger("MosaicAdapterTest"), "Failed to cancel socket operations: %s", cancel_ec.message().c_str());
            }
        }
    });

    // Start the asynchronous receive operation
    socket.async_receive_from(
        boost::asio::buffer(recv_buffer), client_endpoint,
        [&](const boost::system::error_code& ec, std::size_t len) {
            if (ec == boost::asio::error::operation_aborted) {
                RCLCPP_WARN(rclcpp::get_logger("MosaicAdapterTest"), "Receive operation was canceled due to timeout or closure.");
                return;
            }

            if (!socket.is_open()) {
                RCLCPP_ERROR(rclcpp::get_logger("MosaicAdapterTest"), "Socket is not open during receive operation.");
                return;
            }

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
        io_context.run_one(); // Process one event at a time
    }

    // Before exiting, close the socket properly
    if (socket.is_open()) {
        boost::system::error_code ec;
        socket.close(ec);
        if (ec) {
            RCLCPP_ERROR(rclcpp::get_logger("MosaicAdapterTest"), "Error closing socket: %s", ec.message().c_str());
        }
    }

    // Check if the message was received within the timeout period
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
