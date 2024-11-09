/**
 * @file trigger_service_server.cpp
 * @brief Implementation of a Trigger service server node in ROS 2 using rclcpp.
 *
 * This file defines a simple ROS 2 service server node that provides a service
 * called "trigger_service". The service returns a success message to any client
 * that calls it.
 */

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/trigger.hpp"

/**
 * @class TriggerServiceServer
 * @brief A ROS 2 node that provides a trigger service.
 *
 * This class defines a ROS 2 node that offers a service named "trigger_service".
 * When the service is called, it responds with a success status and a message.
 */
class TriggerServiceServer : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for TriggerServiceServer.
     *
     * Initializes the node and creates the trigger service. Logs the readiness
     * of the server once the service is available.
     */
    TriggerServiceServer() : Node("service_server_node")
    {
        service_ = this->create_service<example_interfaces::srv::Trigger>(
            "trigger_service",
            std::bind(&TriggerServiceServer::handle_service, this, std::placeholders::_1, std::placeholders::_2)
        );
        RCLCPP_INFO(this->get_logger(), "Trigger service server is ready.");
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Waiting for client requests");
    }

private:
    /**
     * @brief Callback function for handling service requests.
     *
     * This function is called whenever a client requests the "trigger_service".
     * It sets the response status to success and provides a success message.
     *
     * @param request 
     * @param response The service response containing success status and message.
     */
    void handle_service(
        const std::shared_ptr<example_interfaces::srv::Trigger::Request> request,
        std::shared_ptr<example_interfaces::srv::Trigger::Response> response)
    {
        (void)request;  
        response->success = true;
        response->message = "Trigger service responded successfully!";
        RCLCPP_INFO(this->get_logger(), "Service response sent.");
    }

    rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr service_;  // Shared pointer to the Trigger service.
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TriggerServiceServer>());
    rclcpp::shutdown();
    return 0;
}
