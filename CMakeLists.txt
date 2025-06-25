cmake_minimum_required(VERSION 3.8)
project(robosoccer_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Use legacy Python detection to fix build errors on some systems
find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs 3 REQUIRED)
set(PYTHON_EXECUTABLE ${PYTHON_EXECUTABLE})
set(PYTHON_INCLUDE_DIR ${PYTHON_INCLUDE_DIRS})
set(PYTHON_LIBRARY ${PYTHON_LIBRARIES})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)

# TSID library (proper C++ TSID)
find_package(tsid REQUIRED)

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointTrajectory.msg"
  "msg/TSIDCommand.msg"
  "msg/Footstep.msg"
  "msg/FootstepArray.msg"
  DEPENDENCIES std_msgs geometry_msgs trajectory_msgs
)

# Include directories
include_directories(include)

# Create TSID controller executable
add_executable(tsid_controller_node
  src/tsid_controller_node.cpp
  src/tsid_controller.cpp
)

ament_target_dependencies(tsid_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  pinocchio
  ament_index_cpp
  tsid
)

# Link Eigen3 and TSID
target_link_libraries(tsid_controller_node Eigen3::Eigen tsid::tsid)

# Link message interfaces
rosidl_target_interfaces(tsid_controller_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Add TSID to trajectory bridge
add_executable(tsid_to_trajectory_bridge src/tsid_to_trajectory_bridge.cpp)
ament_target_dependencies(tsid_to_trajectory_bridge 
  rclcpp 
  std_msgs 
  trajectory_msgs
)

# Install targets
install(TARGETS
  tsid_controller_node
  tsid_to_trajectory_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 