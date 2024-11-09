cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(example_interfaces REQUIRED)

# Define executables
add_executable(server src/service_server_node.cpp)
ament_target_dependencies(server rclcpp std_msgs std_srvs example_interfaces)

add_executable(client src/service_client_node.cpp)
ament_target_dependencies(client rclcpp std_msgs std_srvs example_interfaces)

# Install executables
install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# This must be the last statement in CMakeLists.txt
ament_package()
