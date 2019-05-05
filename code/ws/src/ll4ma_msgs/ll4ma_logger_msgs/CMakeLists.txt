cmake_minimum_required(VERSION 2.8.3)
project(ll4ma_logger_msgs)
 
find_package(catkin REQUIRED COMPONENTS
  message_generation
  genmsg
  std_msgs
  sensor_msgs
  geometry_msgs
  reflex_msgs
)

add_message_files(DIRECTORY msg)
add_service_files(DIRECTORY srv)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  reflex_msgs
)

catkin_package(
   CATKIN_DEPENDS
   message_runtime
   std_msgs
   sensor_msgs
   geometry_msgs
   reflex_msgs
)
