
// ros basics
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
// ros types
#include <geometry_msgs/Point.h>

// opencv stuff
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// pcl basics
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PointIndices.h>
#include <pcl/common/transforms.h>
// filters
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

// ransac includes
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//include for finding Surface Normals
#include <pcl/features/integral_image_normal.h>

//include for pcl visualization
#include <pcl/visualization/cloud_viewer.h>

// typedefs
typedef pcl::PointXY PtXY;
typedef pcl::PointCloud<PtXY> PCXY;

typedef pcl::PointXYZ PtXYZ;
typedef pcl::PointCloud<PtXYZ> PCXYZ;

typedef pcl::PointXYZRGB PtXYZRGB;
typedef pcl::PointCloud<PtXYZRGB> PCXYZRGB;

typedef pcl::Normal PtNormal;
typedef pcl::PointCloud<PtNormal> PCNormal;

typedef pcl::visualization::PCLVisualizer PCVisualizer;
typedef pcl::visualization::PointCloudColorHandlerRGBField<PtXYZRGB> PCColorHandler;

//typedef sensor_msgs::PointCloud2 MsgCloud;
typedef Eigen::Matrix<double, 3, 4> ProjMat;

#include <math.h>       /* cos/sin */

#define PI 3.14159265
#include <point_cloud_segmentation/drawEllipsoid.h>

struct EllipsoidParameters
{
  double a;
  double b;
  double c;
  Eigen::Matrix4f transform;
};

class EllipsoidVisualization
{
  ros::NodeHandle n_;
  bool debug_;
  int size_;
  double d_big_,d_small_,d_line_;
  ros::ServiceServer service;
  char r_,g_,b_;
public:
  EllipsoidVisualization(ros::NodeHandle n):
    n_(n)
  {
    ros::NodeHandle n_private("~");
    n_private.param("debug",debug_,true);
    n_private.param("Large_Delta",d_big_,10.0);
    n_private.param("Small_Delta",d_small_,0.5);
    n_private.param("Line_Delta",d_line_,0.0001);
   
    d_big_ = d_big_ * PI/180.0;
    d_small_ = d_small_ * PI/180.0;

    size_ = 3 * PI * (1/d_small_ + 1/d_big_);

    service = n_.advertiseService("draw_ellipsoid", &EllipsoidVisualization::drawEllipsoidSrv,this);
    /*r_ = 172;
    g_ = 153;
    b_ = 193;*/
    r_ = 100;
    g_ = 209;
    b_ = 62;
  }

  inline double dist(PtXYZRGB a, PtXYZRGB b)
  {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  }
  bool drawEllipsoidSrv(point_cloud_segmentation::drawEllipsoid::Request &req,
                        point_cloud_segmentation::drawEllipsoid::Response &res)
  {
    // unpack data
    EllipsoidParameters params;
    params.a = req.a;
    params.b = req.b;
    params.c = req.c;
    // need to get the quaternion and vector from here
    Eigen::Quaternionf rot(req.pose.orientation.w,
                           req.pose.orientation.x,
                           req.pose.orientation.y,
                           req.pose.orientation.z);
    Eigen::Vector3f trans;
    trans << req.pose.position.x, 
      req.pose.position.y, 
      req.pose.position.z;
    params.transform.setIdentity();
    params.transform.block<3,3>(0,0) = rot.normalized().toRotationMatrix();
    params.transform.block<3,1>(0,3) = trans;

    if(debug_)
      std::cout << "Transformation Matrix:\n" << params.transform << std::endl;

    sensor_msgs::ImageConstPtr color_img_msg =
      boost::shared_ptr<sensor_msgs::Image const>
      (new sensor_msgs::Image(req.rgb_img));
    cv::Mat rgb_img = cv_bridge::toCvShare(color_img_msg,"rgb8")->image;

    cv::Mat ellipsoid_img = rgb_img.clone();
    cv::Mat box_img = rgb_img.clone();

    PCXYZRGB::Ptr raw_cloud(new PCXYZRGB);
    pcl::fromROSMsg(req.raw_cloud,*raw_cloud);

    // make ellipsoid
    PCXYZRGB::Ptr ellipsoid(new PCXYZRGB);
    PCXYZRGB::Ptr box(new PCXYZRGB);

    createEllipsoid(ellipsoid,params);
    createBox(box,params);

    ROS_INFO_STREAM("Box Size:" << box->points.size());
    //if(debug_)
    //  displayCloud(box);

    // transform the cloud
    pcl::transformPointCloud(*ellipsoid,*ellipsoid, params.transform);
    ROS_INFO("Transformed ellipsoid");
    pcl::transformPointCloud(*box,*box,params.transform);
    ROS_INFO("Transformed box");
    //if(debug_)
      //displayCloud(ellipsoid);

    // project the ellipsoid back into the image
    cv::imshow("Before",rgb_img);
    projectEllipsoid(ellipsoid,ellipsoid_img,raw_cloud);
    cv::imshow("Ellipsoid Image",ellipsoid_img);
    projectEllipsoid(box,box_img,raw_cloud);

    cv::imshow("Box Image", box_img);
    cv::waitKey(0);

    cv::imwrite("/home/hunter/ws_catkin/results/raw_image.png",rgb_img);
    cv::imwrite("/home/hunter/ws_catkin/results/ellipsoid_image.png",ellipsoid_img);;
    cv::imwrite("/home/hunter/ws_catkin/results/box_image.png",box_img);
    
  }

  PtXYZRGB getPt(double x, double y, double z)
  {
    PtXYZRGB pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;
    return pt;
  }

  void displayPt(PtXYZRGB holder, std::string name="")
  {
    ROS_INFO_STREAM(name << " x:" << holder.x << " y:" << holder.y << " z:" << holder.z);
  }
  void addPts(PtXYZRGB a, PtXYZRGB b, PCXYZRGB::Ptr cloud)
  {
    PtXYZRGB holder(a);
    PtXYZRGB dir;
    dir.x = b.x - a.x;
    dir.y = b.y - a.y;
    dir.z = b.z - a.z;

    double scale = sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
    dir.x = dir.x/scale;
    dir.y = dir.y/scale;
    dir.z = dir.z/scale;

    cloud->points.push_back(a);
    cloud->points.push_back(b);

    while( dist(holder,b) > 0.001)
    {
      PtXYZRGB newPt;
      newPt.x = holder.x + d_line_ * dir.x;
      newPt.y = holder.y + d_line_ * dir.y;
      newPt.z = holder.z + d_line_ * dir.z;
      newPt.r = r_;
      newPt.g = g_;
      newPt.b = b_;
      cloud->points.push_back(newPt);
      holder = newPt;
    }
  }

  void createBox(PCXYZRGB::Ptr box,
                 EllipsoidParameters params)
  {
    box->clear();
    box->height = 1;

    double max_x = params.a/2;
    double max_y = params.b/2;
    double max_z = params.c/2;

    ROS_INFO_STREAM("max_x:" << max_x << " max y:" << max_y << " max z:" << max_z);
    // first define the points
    PtXYZRGB A = getPt(-max_x, max_y, max_z);
    PtXYZRGB B = getPt(max_x, max_y, max_z);
    PtXYZRGB C = getPt(-max_x, -max_y, max_z);
    PtXYZRGB D = getPt(max_x, -max_y, max_z);
    PtXYZRGB E = getPt(-max_x, max_y, -max_z);
    PtXYZRGB F = getPt(-max_x, -max_y, -max_z);
    PtXYZRGB G = getPt(max_x, max_y, -max_z);
    PtXYZRGB H = getPt(max_x, -max_y, -max_z);

    addPts(A,B,box);
    addPts(A,C,box);
    addPts(A,E,box);
    addPts(B,D,box);
    addPts(B,G,box);
    addPts(C,D,box);
    addPts(C,F,box);
    addPts(D,H,box);
    addPts(E,F,box);
    addPts(E,G,box);
    addPts(F,H,box);
    addPts(H,G,box);
      
  }
  void createEllipsoid(PCXYZRGB::Ptr ellipsoid, 
                       EllipsoidParameters params)
  {
    ellipsoid->clear();
    ellipsoid->height = 1;
    ellipsoid->width  = 2*size_;

    double a = params.a/2;
    double b = params.b/2;
    double c = params.c/2;
    for(double theta=-PI/2; theta < PI/2; theta+=d_big_)
    {
      for(double rho=-PI; rho < PI; rho+=d_small_)
      {
        PtXYZRGB pt;
        pt.x = a * cos(theta) * cos(rho);
        pt.y = b * cos(theta) * sin(rho);
        pt.z = c * sin(theta);
        pt.r = r_;
        pt.g = g_;
        pt.b = b_;
        ellipsoid->points.push_back(pt);
      }
    }

    for(double theta=-PI/2; theta < PI/2; theta+=d_small_)
    {
      for(double rho=-PI; rho < PI; rho+=d_big_)
      {
        PtXYZRGB pt;
        pt.x = a * cos(theta) * cos(rho);
        pt.y = b * cos(theta) * sin(rho);
        pt.z = c * sin(theta);
        pt.r = r_;
        pt.g = g_;
        pt.b = b_;
        ellipsoid->points.push_back(pt);
      }
    }
  }

  void projectEllipsoid(PCXYZRGB::Ptr ellipsoid,
                        cv::Mat rgb_img,
                        PCXYZRGB::Ptr raw_cloud)
  {
    double f_ = 570.34222;
    double cx_ = 319.5;
    double cy_ = 239.5;

    int u,v;
    
    PtXYZRGB pt;
    for(int i=0; i < ellipsoid->points.size(); ++i)
    {
      pt = ellipsoid->points[i];
      u = f_/pt.z * pt.x + cx_;
      v = f_/pt.z * pt.y + cy_;
      if(pt.z < raw_cloud->at(u,v).z)
        rgb_img.at<cv::Vec3b>(v,u) = cv::Vec3b(pt.b,pt.g,pt.r);
    }
    
  }

  void displayCloud(PCXYZRGB::Ptr cloud,std::string name="Cloud")
  {
    pcl::visualization::PCLVisualizer viewer("PCL_Viewer");
    viewer.setBackgroundColor(0.0,0.0,0.5);
    // v is view u is up      x   y   z  v_x v_y v_z u_x u_y u_z
    //viewer.setCameraPosition(0.0,0.0,0.0,0.0,0.0,1.0,0.0,-1.0,0.0);
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbA(cloud); 
    viewer.addPointCloud<PtXYZRGB>(cloud,name);
    while ( !viewer.wasStopped() )
      viewer.spinOnce();
  }

};

int main(int argc, char ** argv)
{
  double a = 4.0;
  double b = 8.0;
  double c = 8.0;
  
  
  ros::init(argc, argv,std::string( "Ellipsoid_Visualizer"));
  ros::NodeHandle n;
  EllipsoidVisualization EV(n);
  ros::spin();

  ROS_INFO_STREAM("After ros::spin");
  return 0;
}

/*

rosservice call /draw_ellipsoid "a: 4.0
b: 8.0
c: 16.0
transform:
  translation: {x: 1.0, y: 2.0, z: 3.0}
  rotation: {x: 0.0, y: 0.6502878, z: 0.0, w: -0.7596879}
rgb_img:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  height: 0
  width: 0
  encoding: ''
  is_bigendian: 0
  step: 0
  data: ''
camera_info:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  height: 0
  width: 0
  distortion_model: ''
  D: [0]
  K: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  R: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  P: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  binning_x: 0
  binning_y: 0
  roi: {x_offset: 0, y_offset: 0, height: 0, width: 0, do_rectify: false}" 

*/
