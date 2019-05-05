cmake_minimum_required(VERSION 2.8.3)
project(ll4ma_robots_gazebo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  robot_state_publisher
  roscpp
  rospy
  effort_controllers
  sensor_msgs
  ll4ma_robots_description
  ll4ma_robot_control
  ll4ma_robot_interface
)

find_package(baxter_core_msgs QUIET)

catkin_python_setup()

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_joint_wrapper src/robot_joint_wrapper.cpp)
target_link_libraries(robot_joint_wrapper ${catkin_LIBRARIES})

add_executable(joint_state_splitter src/joint_state_splitter.cpp)
target_link_libraries(joint_state_splitter ${catkin_LIBRARIES})

add_executable(gazebo_control_manager src/control/control_manager.cpp)
target_link_libraries(gazebo_control_manager ${catkin_LIBRARIES})
if (baxter_core_msgs_FOUND)
  target_compile_definitions(gazebo_control_manager PRIVATE BAXTER_DEPENDS_FOUND)
endif()
