cmake_minimum_required(VERSION 3.8)
project(bebop_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

            

add_executable(bebop_node src/bebop.cpp)
ament_target_dependencies(bebop_node rclcpp std_msgs sensor_msgs OpenCV geometry_msgs cv_bridge)

install(TARGETS
  bebop_node
  
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
