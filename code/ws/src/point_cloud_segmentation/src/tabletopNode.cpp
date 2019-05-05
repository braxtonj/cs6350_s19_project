
#include <Tabletop_Segmentation.h>
#include <multiObjectSegmentation.h>
#include <singleObjectSegmentation.h>

// ros basics
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tabletopSegmentation");
  ros::NodeHandle nh;
  
  bool multi;
  nh.param<bool>("Multi_Object_Segmentation", multi, false);
  if(multi)
  {
    PCSeg::tabletop::TabletopSegmentation* segmenter = new PCSeg::tabletop::MultiObjectSegmentation(nh);
    segmenter->spin();
  }
  else
  {
    PCSeg::tabletop::TabletopSegmentation* segmenter = new PCSeg::tabletop::SingleObjectSegmentation(nh);
    segmenter->spin();
  }
  ROS_INFO("About to call spin");
  //segmenter->spin();

}
