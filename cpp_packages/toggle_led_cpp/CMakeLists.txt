cmake_minimum_required(VERSION 3.8)
project(toggle_led_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(led_interface REQUIRED)

# add executables
add_executable(server_led_panel src/server_led_panel.cpp)
ament_target_dependencies(server_led_panel rclcpp led_interface)
add_executable(battery_state src/battery_state.cpp)
ament_target_dependencies(battery_state rclcpp led_interface)


install(TARGETS
  server_led_panel
  battery_state
  
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
