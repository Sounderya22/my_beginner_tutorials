/**
 * @file trigger_service_server.cpp
 * @brief Implementation of a Trigger service server node in ROS 2 using rclcpp.
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
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

/**
 * @class TriggerServiceServer
 * @brief A ROS 2 node that provides a trigger service and broadcasts a static transform.
 */
class TriggerServiceServer : public rclcpp::Node {
 public:
    /**
     * @brief Constructor for TriggerServiceServer.
     *
     * Initializes the node, creates the trigger service, and broadcasts a static transform.
     */
    TriggerServiceServer() : Node("talker") {
        // Create the Trigger service
        service_ = this->create_service<example_interfaces::srv::Trigger>(
            "trigger_service",
            std::bind(&TriggerServiceServer::handleService,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

        // Initialize the StaticTransformBroadcaster
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Broadcast the static transform
        publishStaticTransform();

        RCLCPP_INFO(this->get_logger(), "Trigger service server is ready.");
    }

 private:
    /**
     * @brief Publishes a static transform between /world and /talk.
     */
    void publishStaticTransform() {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "world";  // Parent frame
        transform.child_frame_id = "talk";   // Child frame

        // Define non-zero translation
        transform.transform.translation.x = 1.0;
        transform.transform.translation.y = 2.0;
        transform.transform.translation.z = 0.5;

        // Define non-zero rotation as a quaternion (90 degrees about Z-axis)
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.707;  // sin(45°)
        transform.transform.rotation.w = 0.707;  // cos(45°)

        // Send the static transform
        tf_static_broadcaster_->sendTransform(transform);
        RCLCPP_INFO(this->get_logger(), "Static transform /world -> /talk broadcasted.");
    }

    /**
     * @brief Callback function for handling service requests.
     *
     * This function is called whenever a client requests the "trigger_service".
     * It sets the response status to success and provides a success message.
     *
     * @param request 
     * @param response The service response containing success status and message.
     */
    void handleService(
        const std::shared_ptr<example_interfaces::srv::Trigger::Request> request,
        std::shared_ptr<example_interfaces::srv::Trigger::Response> response) {
        (void)request;
        response->success = true;
        response->message = "Trigger service responded successfully!";
        RCLCPP_INFO(this->get_logger(), "Service response sent.");
    }

    rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr service_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TriggerServiceServer>());
    rclcpp::shutdown();
    return 0;
}
