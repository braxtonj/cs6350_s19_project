cmake_minimum_required(VERSION 2.8.3)
project(ll4ma_robot_control_msgs)
 
find_package(catkin REQUIRED COMPONENTS
  message_generation
  genmsg
  std_msgs
  geometry_msgs
  sensor_msgs
)

add_message_files(
  FILES
  RobotState.msg
  CartesianCommand.msg
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
   std_msgs
   geometry_msgs
   sensor_msgs
)
