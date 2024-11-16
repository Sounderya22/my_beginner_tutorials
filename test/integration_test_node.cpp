/**
 * @file integration_test_node.cpp
 * @brief This file creates a test node that checks if messages are being successfully published to the /chatter topic.
 * 
 * This integration test node subscribes to the `/chatter` topic and verifies if it receives messages within a specified test duration. 
 * It uses the Catch2 framework for testing.
 * 
 * MIT License
 * 
 * Copyright 2023 Nick Morales.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * # CHANGES:
 * 
 * 2024-10-31, Sounderya Varagur Venugopal
 *     - Added a test case, "test talker".
 *     - Use modern ROS2 syntax
 *     - Use Catch2 Fixture
 */

#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using std_msgs::msg::String;

////////////////////////////////////////////////
// Define Fixture
////////////////////////////////////////////////

/**
 * @brief Logger for ROS 2 nodes.
 * 
 * This logger is used throughout the test to log important information
 * related to the test execution.
 */
auto logger = rclcpp::get_logger("");  // Create an initial Logger

/**
 * @class MyTestsFixture
 * @brief A fixture class to set up and tear down the testing environment.
 * 
 * This fixture handles the creation of the test node, parameter declarations,
 * and the test duration setup. It is used to set up the context for the test 
 * cases and ensure proper initialization and cleanup.
 */
class MyTestsFixture {
 public:
  /**
   * @brief Constructor for the MyTestsFixture class.
   * 
   * Initializes the ROS node and declares the `test_duration` parameter.
   * The constructor also retrieves the test duration parameter from the ROS node
   * for use in the test cases.
   */
  MyTestsFixture() {
    // Create the node that performs the test (Integration test node)
    tester_node_ = rclcpp::Node::make_shared("IntegrationTestNode1");
    logger = tester_node_->get_logger();  // Ensure message will appear in rqt_console

    // Declare a parameter for the duration of the test
    tester_node_->declare_parameter<double>("test_duration");

    // Get the test duration value
    test_duration_ = tester_node_->get_parameter("test_duration").get_parameter_value().get<double>();
    RCLCPP_INFO_STREAM(logger, "Got test_duration =" << test_duration_);
  }

  /**
   * @brief Destructor for the MyTestsFixture class.
   * 
   * Currently no specific cleanup is needed, but the destructor is included
   * for completeness.
   */
  ~MyTestsFixture() {}

 protected:
  double test_duration_;  ///< Duration for the test
  rclcpp::Node::SharedPtr tester_node_;  ///< Shared pointer to the ROS node performing the test
};

////////////////////////////////////////////////
// Test Case 1
////////////////////////////////////////////////

/**
 * @brief Test case that checks if the topic `/chatter` receives messages within the specified duration.
 * 
 * In this test case, the node subscribes to the `/chatter` topic, and the test 
 * simply checks if the topic message is received within the `test_duration` period.
 * If the message is received, the test passes; otherwise, it fails.
 */
TEST_CASE_METHOD(MyTestsFixture, "test talker", "[topic]") {
  /**
   * @brief Flag indicating whether the topic message was received.
   */
  bool got_topic = false;

  /**
   * @brief Callback structure for subscribing to the `/chatter` topic.
   * 
   * This callback is invoked whenever a message is received on the `/chatter` topic.
   * It sets the `got_topic` flag to `true` and logs the received message.
   */
  struct ListenerCallback {
    explicit ListenerCallback(bool &got_topic) : got_topic_(got_topic) {}
    
    void operator()(const String msg) const {
      RCLCPP_INFO_STREAM(logger, "I heard: " << msg.data.c_str());
      got_topic_ = true;  // Set the flag to true once a message is received
    }

    bool &got_topic_;  ///< Reference to the flag indicating if the topic message is received
  };

  // Create a subscriber to the "chatter" topic
  auto subscriber = tester_node_->create_subscription<String>("chatter", 10, ListenerCallback(got_topic));

  /**
   * @brief Perform the actual test.
   * 
   * The test checks if the topic message is received within the specified test duration.
   * It spins the node at 10 Hz until either the topic message is received or the time runs out.
   */
  rclcpp::Rate rate(10.0);  ///< Check every 10 Hz
  auto start_time = rclcpp::Clock().now();  ///< Get the current time
  auto duration = rclcpp::Clock().now() - start_time;  ///< Duration since the test started
  auto timeout = rclcpp::Duration::from_seconds(test_duration_);  ///< Test duration timeout

  RCLCPP_INFO_STREAM(logger, "duration = " << duration.seconds() << " timeout=" << timeout.seconds());

  // Spin until the message is received or the timeout occurs
  while (!got_topic && (duration < timeout)) {
    rclcpp::spin_some(tester_node_);  ///< Spin the node and process callbacks
    rate.sleep();  ///< Sleep for a short period (to maintain 10 Hz rate)
    duration = (rclcpp::Clock().now() - start_time);  ///< Update the elapsed duration
  }

  RCLCPP_INFO_STREAM(logger, "duration = " << duration.seconds() << " got_topic=" << got_topic);
  CHECK(got_topic);  ///< Assert that the topic message was received
}
