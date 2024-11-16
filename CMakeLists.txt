cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Enable compile_commands.json
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)  
find_package(tf2_ros REQUIRED)
find_package(catch_ros2 REQUIRED)  # Use catch_ros2 for ROS 2 tests
find_package(rosbag2_cpp REQUIRED)

if(BUILD_TESTING)
  # disable these built-in tests
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # build the integration test
  ###########################################################################
  # 1.) BUILD AND INSTALL THE INTEGRATION TEST NODE (aka Node that performs the test)  ##
  ###########################################################################
  # This is the node in which integration tests occur
  add_executable(integration_test_node
    test/integration_test_node.cpp
    )
  # The link libraries call this node with "catch_ros2::catch_ros2_with_node_main"
  # to get the default integration test node main function
  target_link_libraries(integration_test_node
    catch_ros2::catch_ros2_with_node_main
    )
  ament_target_dependencies(integration_test_node
    rclcpp std_srvs std_msgs
    )
  install(TARGETS
    integration_test_node
    DESTINATION lib/${PROJECT_NAME}
  )

  ######################################
  ## 2.) ADD INTEGRATION TEST SUPPORT ##
  ######################################
  # This CMake function allows the integration test to be run
  # when running "colcon test".
  catch_ros2_add_integration_test (ExampleIntegration_TestYAML
    LAUNCH_FILE integration_test.launch.yaml  # example in YAML
    )
  
endif()

# Define executables
add_executable(talker src/talker_node.cpp)
ament_target_dependencies(talker rclcpp std_msgs std_srvs example_interfaces geometry_msgs tf2 tf2_ros)

add_executable(listener src/listener_node.cpp)
ament_target_dependencies(listener rclcpp std_msgs std_srvs example_interfaces geometry_msgs tf2 tf2_ros)


# Install executables
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# This must be the last statement in CMakeLists.txt
ament_package()

