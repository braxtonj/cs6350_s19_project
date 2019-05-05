cmake_minimum_required(VERSION 2.8.3)
project(ll4ma_robot_interface)
 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# NOTE: Any robot-specific dependencies are to be conditionally included, as we don't want to
#       require a user to satisfy dependencies for a robot they are not going to use.


find_package(catkin REQUIRED COMPONENTS
  roscpp
  ll4ma_kdl
  ll4ma_robot_control_msgs
)

find_package(Eigen3 REQUIRED)
find_package(baxter_core_msgs QUIET)

set(LRI_CATKIN_DEPENDS
  ll4ma_kdl
  ll4ma_robot_control_msgs
)

set(LRI_INCLUDE_DIRS
  include
)

set(LRI_LIBRARIES
  ll4ma_robot_interface
  lbr4_robot_interface
  allegro_robot_interface
  reflex_robot_interface
)

if(baxter_core_msgs_FOUND)
  list(APPEND LRI_CATKIN_DEPENDS baxter_core_msgs)
  list(APPEND LRI_INCLUDE_DIRS include/baxter_robot_interface ${baxter_core_msgs_INCLUDE_DIRS})
  list(APPEND LRI_LIBRARIES baxter_robot_interface)
else()
  message("Not compiling for use with Baxter robot: missing dependency 'baxter_core_msgs'.")
endif()

catkin_package(
  INCLUDE_DIRS ${LRI_INCLUDE_DIRS}
  LIBRARIES ${LRI_LIBRARIES}
  CATKIN_DEPENDS ${LRI_CATKIN_DEPENDS}
)

include_directories(
  ${LRI_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(ll4ma_robot_interface src/ll4ma_robot_interface/robot_interface.cpp)
target_link_libraries(ll4ma_robot_interface ${catkin_LIBRARIES})

add_library(lbr4_robot_interface src/lbr4_robot_interface/lbr4_interface.cpp)
target_link_libraries(lbr4_robot_interface ${catkin_LIBRARIES})

add_library(allegro_robot_interface src/allegro_robot_interface/allegro_interface.cpp)
target_link_libraries(allegro_robot_interface ${catkin_LIBRARIES})

add_library(reflex_robot_interface src/reflex_robot_interface/reflex_interface.cpp)
target_link_libraries(reflex_robot_interface ${catkin_LIBRARIES})

if(baxter_core_msgs_FOUND)
  add_library(baxter_robot_interface src/baxter_robot_interface/baxter_interface.cpp)
  target_link_libraries(baxter_robot_interface ${catkin_LIBRARIES})
endif()
