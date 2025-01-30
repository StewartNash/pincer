cmake_minimum_required(VERSION 3.8)
project(arctos_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

# find dependencies
find_package(ament_cmake REQUIRED
  COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    message_generation
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    moveit_visual_tools
)

find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Boost REQUIRED system filesystem date_time thread)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#  ArmJointState.msg
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  sensor_msgs
#  std_msgs
#)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmJointState.msg"
  DEPENDENCIES
  sensor_msgs
  std_msgs
 )

ament_package(
  AMENT_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${ament_INCLUDE_DIRS})
link_directories(${ament_LIBRARY_DIRS})

## Declare a C++ executable
add_executable(moveit_convert src/moveit_convert.cpp)
add_executable(move_group_1 src/move_group_interface_coor_1.cpp)

add_dependencies(moveit_convert moveo_moveit_generate_messages_cpp)
## Specify libraries to link a library or executable target against
ament_install_python(PROGRAMS scripts/moveo_objrec_publisher.py
  DESTINATION ${AMENT_PACKAGE_BIN_DESTINATION})

target_link_libraries(moveit_convert
  ${ament_LIBRARIES}
)
target_link_libraries(move_group_1
  ${ament_LIBRARIES}
)
