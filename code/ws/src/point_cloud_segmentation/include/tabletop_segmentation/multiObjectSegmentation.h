
#ifndef MULTI_OBJECT_SEGMENTATION_H_
#define MULTI_OBJECT_SEGMENTATION_H_

// ros basics
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

// opencv stuff
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// pcl basics
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// Parent Class
#include <Tabletop_Segmentation.h>
// msg files
#include <point_cloud_segmentation/MultiObjectFilter.h>

typedef point_cloud_segmentation::MultiObjectFilter filterMsg;


namespace PCSeg
{
  namespace tabletop
  {

  class MultiObjectSegmentation:public TabletopSegmentation
  {
public:
    MultiObjectSegmentation(ros::NodeHandle n);

    void spin();
    void basicSpin();
    void motionSpin();
    
  };

  }
}

#endif // include protect



