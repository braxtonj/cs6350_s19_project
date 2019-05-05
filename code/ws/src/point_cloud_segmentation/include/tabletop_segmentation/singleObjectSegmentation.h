
#ifndef SINGLE_OBJECT_SEGMENTATION_H_
#define SINGLE_OBJECT_SEGMENTATION_H_

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


namespace PCSeg
{
  namespace tabletop
  {

  class SingleObjectSegmentation:public TabletopSegmentation
  {
public:
    SingleObjectSegmentation(ros::NodeHandle n);

    void spin();
    
  };

  }
}

#endif // include protect
