cmake_minimum_required(VERSION 3.8)
project(cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(robot_interface REQUIRED)
find_package(project_turtle_interface)
find_package(turtlesim)
find_package(mavros_msgs REQUIRED)

add_executable(cpp_node src/cpp_node.cpp)
ament_target_dependencies(cpp_node rclcpp)
add_executable(robot_station src/robot_station.cpp)
ament_target_dependencies(robot_station rclcpp example_interfaces)
add_executable(smartphone src/smartphone.cpp)
ament_target_dependencies(smartphone rclcpp example_interfaces)
add_executable(server_add_two_ints src/server_add_two_ints.cpp)
ament_target_dependencies(server_add_two_ints rclcpp example_interfaces)
add_executable(client_add_two_ints src/client_add_two_ints.cpp)
ament_target_dependencies(client_add_two_ints rclcpp example_interfaces)
add_executable(class_client_add_two_ints src/class_client_add_two_ints.cpp)
ament_target_dependencies(class_client_add_two_ints rclcpp example_interfaces)
add_executable(robot_hardware_status src/robot_hardware_status.cpp)
ament_target_dependencies(robot_hardware_status rclcpp robot_interface)
add_executable(subs_hardware_status src/subs_hardware_status.cpp)
ament_target_dependencies(subs_hardware_status rclcpp robot_interface)
add_executable(resistance_server src/resistance_server.cpp)
ament_target_dependencies(resistance_server rclcpp robot_interface)
add_executable(resistance_client src/resistance_client.cpp)
ament_target_dependencies(resistance_client rclcpp robot_interface)
add_executable(spawner src/spawner.cpp)
ament_target_dependencies(spawner rclcpp project_turtle_interface turtlesim)

install(TARGETS
  cpp_node
  robot_station
  smartphone
  server_add_two_ints
  client_add_two_ints
  class_client_add_two_ints
  robot_hardware_status
  subs_hardware_status
  resistance_server
  resistance_client
  spawner
  
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
