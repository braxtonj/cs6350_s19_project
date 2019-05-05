
#ifndef TABLETOP_SEGMENTATION_H_
#define TABLETOP_SEGMENTATION_H_

// ros basics
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

// opencv stuff
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// ros messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>

// pcl basics
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// ransac includes
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// include utilites
#include <utils.h>


namespace PCSeg
{

  namespace tabletop
  {
  
  class TabletopSegmentation
  {
  public:
    // decision params
    bool debug_;
    bool send_motion_;
    bool send_object_masks_;
    bool send_object_bb_; // bounding box (not in multi)
    bool keep_organized_;
    bool find_one_plane_;
    bool send_table_;
    bool send_bg_;
    bool use_srv_;
    bool use_topics_;
    bool send_color_imgs_;
    bool send_raw_data_;
    // table parameters
    int ransac_max_iter_;
    double ransac_dist_thresh_;
    double dpt_scale_;
    
    // motion parameters
    int morph_size_, min_motion_size_;
    double max_motion_, max_color_;

    // table filtering
    double filter_bias_;
    double max_z_, min_z_;
    
    // camera data
    sensor_msgs::CameraInfo cam_info_;
    double f_, cx_, cy_;
  
    // ros stuff
    ros::NodeHandle n_;
    ros::Subscriber cloud_sub_;
    ros::Publisher proc_pub_;
    ros::Rate loop_rate_;

    //global data
    bool got_data_, init_;
    PCXYZRGB::Ptr raw_cloud_, old_raw_cloud_;
    double a_,b_,c_,d_;
    
    // useful debugging tools
    std::vector<cv::Vec3b> colors_; 

    TabletopSegmentation(ros::NodeHandle n);
    void cloudCB(const sensor_msgs::PointCloud2ConstPtr&);
    // in_cloud, table indicies, 
    void findPlane(PCXYZRGB::Ptr, pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr);
    void filterPlane(PCXYZRGB::Ptr,PCXYZRGB::Ptr);
    void filterPlane(PCXYZRGB::Ptr,PCNormal::Ptr,PCXY::Ptr,
                     PCXYZRGB::Ptr,PCNormal::Ptr,PCXY::Ptr);
    void filterPlaneKeepTableBG(PCXYZRGB::Ptr in_cloud, PCNormal::Ptr in_nrm, PCXY::Ptr in_uv,
                                PCXYZRGB::Ptr objs_cloud, PCNormal::Ptr objs_nrm, PCXY::Ptr objs_uv,
                               PCXYZRGB::Ptr table_cloud, PCNormal::Ptr table_nrm, PCXY::Ptr table_uv,
                                PCXYZRGB::Ptr bg_cloud, PCNormal::Ptr bg_nrm, PCXY::Ptr bg_uv);
    void filterPlaneMotion(PCXYZRGB::Ptr in_cloud,PCNormal::Ptr in_nrm,PCXY::Ptr in_uv, 
                           cv::Mat motion_mat,
                           PCXYZRGB::Ptr motion_cloud,PCNormal::Ptr motion_nrm, PCXY::Ptr motion_uv,
                           PCXYZRGB::Ptr objs_cloud,  PCNormal::Ptr objs_nrm, PCXY::Ptr objs_uv);

    void filterPlaneMotionKeepTableBG(PCXYZRGB::Ptr in_cloud,
                                      PCNormal::Ptr in_nrm,
                                      PCXY::Ptr in_uv,
                                      cv::Mat motion_mat,
                                      PCXYZRGB::Ptr motion_cloud,
                                      PCNormal::Ptr motion_nrm, 
                                     PCXY::Ptr motion_uv,
                                     PCXYZRGB::Ptr objs_cloud,  
                                     PCNormal::Ptr objs_nrm, 
                                     PCXY::Ptr objs_uv,
                                     PCXYZRGB::Ptr table_cloud, 
                                     PCNormal::Ptr table_nrm, 
                                     PCXY::Ptr table_uv,
                                     PCXYZRGB::Ptr bg_cloud, 
                                     PCNormal::Ptr bg_nrm, 
                                      PCXY::Ptr bg_uv);

    void subtractCloud(PCXYZRGB::Ptr, PCXYZRGB::Ptr, cv::Mat&);
    void getColorVec();
    // virtual functions for spacific use cases
    virtual void spin(void) = 0;

  };
  }
}




#endif //TABLETOP_SEGMENTATION_H_
