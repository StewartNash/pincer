cmake_minimum_required(VERSION 3.8)
project(arctos_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(roscpp REQUIRED)
find_package(rospy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

# Linting (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmJointState.msg"
  DEPENDENCIES sensor_msgs std_msgs
)

# Include directories
include_directories(
  SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS}
)

# Executables
add_executable(moveit_convert src/moveit_convert.cpp)
add_executable(move_group_1 src/move_group_interface_coor_1.cpp)

# Link dependencies
ament_target_dependencies(moveit_convert
  rclcpp
  std_msgs
  sensor_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  moveit_visual_tools
)

ament_target_dependencies(move_group_1
  rclcpp
  std_msgs
  sensor_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  moveit_visual_tools
)

# Install scripts
install(PROGRAMS
  scripts/moveo_objrec_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

