
//opencv includes
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>

#include <std_msgs/Float64.h>

//include for pcl visualization
#include <pcl/visualization/cloud_viewer.h>
typedef pcl::PointXYZRGB PtXYZRGB;
typedef pcl::PointCloud<PtXYZRGB> PCXYZRGB;

// msg files
#include <point_cloud_segmentation/MultiObjectFilter.h>
typedef point_cloud_segmentation::MultiObjectFilter filterMsg;

ros::Publisher pub_cloud;
ros::Publisher pub_objs_cloud;
ros::Publisher pub_objs_motion;
ros::Publisher pub_motion_mask;
ros::Publisher pub_table_cloud;
ros::Publisher pub_bg_cloud;
ros::Publisher pub_tp;



void pubCallback(const filterMsg::ConstPtr& msg)
{
  pub_cloud.publish(msg->cloud);
  pub_objs_cloud.publish(msg->objs_cloud);
  pub_objs_motion.publish(msg->objs_motion_cloud);
  pub_motion_mask.publish(msg->motion_mask);
  pub_table_cloud.publish(msg->table_cloud);
  pub_bg_cloud.publish(msg->bg_cloud);
  std::cout << msg->table_params[0] << "," << msg->table_params[1] << "," << msg->table_params[2] << "," << msg->table_params[3] << std::endl;
  //pub_tp.publish(msg->table_params[0]);
}


int main(int argc, char ** argv)
{

  ros::init(argc, argv, "Topic_Splitter");
  ros::NodeHandle n;
  
  pub_cloud = n.advertise<sensor_msgs::PointCloud2>("raw_cloud",1);
  pub_objs_cloud = n.advertise<sensor_msgs::PointCloud2>("Objects_Cloud",1);
  pub_objs_motion = n.advertise<sensor_msgs::PointCloud2>("Motion_Cloud",1);
  pub_motion_mask = n.advertise<sensor_msgs::Image>("Motion_Mask",1);
  pub_bg_cloud = n.advertise<sensor_msgs::PointCloud2>("Background_Cloud",1);
  pub_table_cloud = n.advertise<sensor_msgs::PointCloud2>("Table_Cloud",1);
  pub_tp          = n.advertise<std_msgs::Float64>("Table_Params",1);
  ros::Subscriber sub = n.subscribe("Multi_Object_Filtered", 1, pubCallback);
  ros::spin();
}

