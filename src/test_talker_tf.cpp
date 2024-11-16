#include "catch_ros2/catch.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

TEST_CASE("Test the /talker to /world transform", "[tf]") {
    // ROS 2 initialization
    rclcpp::init(0, nullptr);
    auto node = rclcpp::Node::make_shared("test_node");
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // Allow time for the transform to be published
    rclcpp::sleep_for(std::chrono::seconds(1));

    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer.lookupTransform("world", "talk", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
        FAIL("Transform not found: " << ex.what());
    }

    // Validate the transform is not zero
    REQUIRE(transform.transform.translation.x != 0.0);
    REQUIRE(transform.transform.translation.y != 0.0);
    REQUIRE(transform.transform.translation.z != 0.0);
    REQUIRE(transform.transform.rotation.x != 0.0);
    REQUIRE(transform.transform.rotation.y != 0.0);
    REQUIRE(transform.transform.rotation.z != 0.0);
    REQUIRE(transform.transform.rotation.w != 0.0);
}
