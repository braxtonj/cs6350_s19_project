cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_segmentation)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  genmsg
  image_transport
  message_filters
  sensor_msgs
  geometry_msgs
  pcl_ros
  tf
  message_generation
  cmake_modules
)

find_package(Eigen REQUIRED)

## Generate services in the 'srv' folder
add_message_files(
  DIRECTORY msg
  FILES
  GraspObject.msg
  MultiObjectFilter.msg
)


add_service_files(
  DIRECTORY srv
  FILES
  FilterPCL.srv
  SegmentGraspObject.srv
  GetVisualData.srv
  drawEllipsoid.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/utils
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tabletop_segmentation
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(PC_Seg_utils src/utils/utils.cpp)
target_link_libraries(PC_Seg_utils  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_library(PC_Seg_Tabletop src/tabletop_segmentation/Tabletop_Segmentation.cpp 
  src/tabletop_segmentation/multiObjectSegmentation.cpp src/tabletop_segmentation/singleObjectSegmentation.cpp)
target_link_libraries(PC_Seg_Tabletop PC_Seg_utils ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include/utils include include/tabletop_segmentation
  LIBRARIES PC_Seg_utils PC_Seg_Tabletop
  CATKIN_DEPENDS message_runtime sensor_msgs roscpp rospy 
  DEPENDS system_lib Eigen
)

add_executable(tabletop_object_segmenter src/table_object_segmenter.cpp)
add_dependencies(tabletop_object_segmenter point_cloud_segmentation_generate_messages_cpp)
target_link_libraries(tabletop_object_segmenter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

## Testing nodes
add_executable(test_utils src/tests/test_utils.cpp)
target_link_libraries(test_utils PC_Seg_utils ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_executable(table_top_segmentation_node src/tabletopNode.cpp)
add_dependencies(table_top_segmentation_node point_cloud_segmentation_generate_messages_cpp)
target_link_libraries(table_top_segmentation_node PC_Seg_utils PC_Seg_Tabletop ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}) 

add_executable(multi_topic_splitter src/tabletop_segmentation/multiTopicSplitter.cpp)
add_dependencies(multi_topic_splitter point_cloud_segmentation_generate_messages_cpp)
target_link_libraries(multi_topic_splitter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}) 

add_executable(ellipsoid_visualization src/ellipsoid_visualization.cpp)
add_dependencies(ellipsoid_visualization point_cloud_segmentation_generate_messages_cpp)
target_link_libraries(ellipsoid_visualization ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
