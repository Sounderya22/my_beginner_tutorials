/**
 * @file service_client_node.cpp
 * @brief A ROS 2 client node that calls a Trigger service at regular intervals.
 * 
 * MIT License
 * 
 * Copyright (c) [year] [your name or organization]
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/trigger.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

/**
 * @class TriggerServiceClient
 * @brief A ROS 2 node that periodically calls a Trigger service.
 * 
 * The TriggerServiceClient class creates a client node that attempts to call
 * a service named "trigger_service" every second. If the service is not 
 * available after 1 minute of attempts, the node logs a fatal message and shuts down.
 */
class TriggerServiceClient : public rclcpp::Node {
 public:
    /**
     * @brief Constructor for the TriggerServiceClient node.
     * 
     * Initializes the ROS client for the Trigger service and sets up a timer to
     * periodically call the service.
     */
    TriggerServiceClient() : Node("service_client"), wait_duration_(0s) {
        // Create a client for the "trigger_service" service
        client_ = this->create_client<example_interfaces::srv::Trigger>("trigger_service");

        // Set up a timer to call the service every second.
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&TriggerServiceClient::callService, this));
    }

 private:
    /**
     * @brief Calls the Trigger service.
     * 
     * This function is called every second by the timer. It attempts to connect 
     * to the service, and if unsuccessful, waits up to 1 second. If the service
     * has been unavailable for over a minute, a fatal message is logged and the
     * node shuts down.
     */
    void callService() {
        // Increment wait duration and check if a minute has passed
        wait_duration_ += 1s;
        if (wait_duration_ >= 60s) {
            RCLCPP_FATAL_STREAM(
              this->get_logger(),
              "Service unavailable for over 1 minute. Shutting down");
            rclcpp::shutdown();
            return;
        }

        // Try to connect to the service
        if (!client_->wait_for_service(1s)) {
            RCLCPP_WARN_STREAM(
              this->get_logger(), "Waiting for the service...");
            return;
        }

        // Reset wait duration if service is available
        wait_duration_ = 0s;

        // Prepare the request for the Trigger service
        auto request = std::make_shared<example_interfaces::srv::Trigger::Request>();

        // Send the request and define a callback for the response
        auto future = client_->async_send_request(request, [this](
          rclcpp::Client<example_interfaces::srv::Trigger>::SharedFuture future_response) {
                auto response = future_response.get();
                if (response->success) {
                    RCLCPP_INFO(
                      this->get_logger(), "Received response: %s", response->message.c_str());
                } else {
                    RCLCPP_ERROR_STREAM(
                      this->get_logger(), "Service call failed.");
                }
            });
    }

    rclcpp::Client<example_interfaces::srv::Trigger>::SharedPtr client_;  // Client for Trigger service
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::seconds wait_duration_;
};

/**
 * @brief Main function to run the TriggerServiceClient node.
 * 
 * Initializes the ROS 2 system, creates a shared pointer to the node, and spins
 * until shutdown is called.
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TriggerServiceClient>());
    rclcpp::shutdown();
    return 0;
}
