cmake_minimum_required(VERSION 3.8)
project(biped_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(biped_controller_node 
  src/biped_controller_node.cpp
  src/biped_controller_utils.cpp
)

ament_target_dependencies(biped_controller_node 
  rclcpp 
  std_msgs
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  biped_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(include msg)

ament_package()
