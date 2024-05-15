cmake_minimum_required(VERSION 3.8)
project(flying_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)

add_executable(circle src/circle.cpp)
ament_target_dependencies(circle rclcpp mavros_msgs)

install(TARGETS
  circle
  
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
