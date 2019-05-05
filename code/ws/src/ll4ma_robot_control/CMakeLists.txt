cmake_minimum_required(VERSION 2.8.3)
project(ll4ma_robot_control)
 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  genmsg
  std_msgs
  geometry_msgs
  sensor_msgs
  tf
  robot_state_publisher
  ll4ma_robot_interface
  ll4ma_trajectory_msgs
)

catkin_python_setup()

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  OpSpaceCommand.msg
  Constraint.msg
)

add_service_files(
  FILES
  SwitchControl.srv
  JointPosition.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  ll4ma_robot_interface
  INCLUDE_DIRS include
  LIBRARIES ll4ma_robot_control
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(ll4ma_robot_control
  src/ll4ma_robot_control/controller_manager.cpp
  src/ll4ma_robot_control/control.cpp
  src/ll4ma_robot_control/joint_pd_control.cpp
  src/ll4ma_robot_control/task_inv_dyn_control.cpp
)
add_dependencies(ll4ma_robot_control ${PROJECT_NAME}_generate_messages_cpp)
 
add_library(base_control src/ll4ma_robot_control/control.cpp)
add_dependencies(base_control ${PROJECT_NAME}_generate_messages_cpp)

add_library(task_velocity_control
  src/ll4ma_robot_control/control.cpp
  src/ll4ma_robot_control/task_velocity_control.cpp
)
add_dependencies(task_velocity_control ${PROJECT_NAME}_generate_messages_cpp)

add_executable(lbr4_task_velocity_controller
   src/ll4ma_robot_control/lbr4/lbr4_velocity_control.cpp
)
target_link_libraries(lbr4_task_velocity_controller task_velocity_control ${catkin_LIBRARIES})
add_dependencies(lbr4_task_velocity_controller ${PROJECT_NAME}_generate_messages_cpp)
