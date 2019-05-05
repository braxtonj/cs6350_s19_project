cmake_minimum_required(VERSION 2.8.3)
project(ll4ma_kdl)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(DART 6.1  COMPONENTS gui utils utils-urdf)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED message_generation
std_msgs
 COMPONENTS
 roscpp
 kdl_parser
 roslib
 eigen_conversions
 geometry_msgs
 tf_conversions
)

add_message_files(
  DIRECTORY msg
  FILES
  IK_command.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include

${CMAKE_CURRENT_SOURCE_DIR}/include/ll4ma_kdl
${CMAKE_CURRENT_SOURCE_DIR}/include/ll4ma_kdl/manipulator_kdl
${DART_INCLUDE_DIRS}
)
add_definitions(${DART_DEFINITIONS})


## Declare C++ libraries


catkin_package(
  INCLUDE_DIRS include include/ll4ma_kdl
  LIBRARIES manipulator_kdl manipulator_kdl_bck transforms
  CATKIN_DEPENDS
  kdl_parser
  eigen_conversions
  message_runtime 
  message_generation
  std_msgs
  geometry_msgs
  tf_conversions
  DEPENDS 
)
add_library(manipulator_kdl src/manipulator_kdl/robot_kdl.cpp)
add_library(manipulator_kdl_bck src/manipulator_kdl/robot_kdl_bck.cpp)
add_library(transforms src/utils/transforms.cpp)
###########
## Build ##
###########



## Declare executables:
add_executable(kdl_test_node src/kdl_test_node.cpp)
#add_executable(benchmark_kdl src/benchmark_kdl.cpp)

## Link libraries
target_link_libraries(kdl_test_node manipulator_kdl ${catkin_LIBRARIES} )
#target_link_libraries(benchmark_kdl manipulator_kdl ${catkin_LIBRARIES})


#add_executable(dart_kdl_test src/dart_kdl_test.cpp)
#target_link_libraries(dart_kdl_test   manipulator_kdl ${DART_LIBRARIES} ${catkin_LIBRARIES})
#add_executable(dart_test src/dart_test_node.cpp)
#target_link_libraries(dart_test ${DART_LIBRARIES})
