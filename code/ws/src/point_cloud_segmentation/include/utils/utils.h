#ifndef PCSEG_UTILS_H_
#define PCSEG_UTILS_H_
// ros types
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
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
// filters
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/pca.h>

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

namespace PCSeg
{

  namespace utils
  {

  class ProtoObject
  {
  public:
    PCXYZ::Ptr cloud;
    PCNormal::Ptr normals;
    Eigen::Vector4f centroid;
    int id;
    bool moved;
    Eigen::Matrix4f transform;
    std::vector<int> boundary_angle_dist;
    std::vector<int> push_history;
    bool singulated;
    double icp_score;
  };

  typedef std::deque<ProtoObject> ProtoObjects;

  inline bool notNaN(PtNormal nrm)
  {
    return ( !std::isnan(nrm.normal[0]) && !std::isnan(nrm.normal[1]) && !std::isnan(nrm.normal[2]) );
  }

  inline bool notNaN(PtXYZRGB pt)
  {
    return ( !std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z) );
  }

  inline double dist(PtXYZ a, PtXYZ b)
  {
    const double dx = a.x-b.x;
    const double dy = a.y-b.y;
    const double dz = a.z-b.z;
    return std::sqrt(dx*dx+dy*dy+dz*dz);
  }

  inline double dist(PtXYZRGB a, PtXYZRGB b)
  {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  }

  inline double dist(PtXYZ a, geometry_msgs::Point b)
  {
    const double dx = a.x-b.x;
    const double dy = a.y-b.y;
    const double dz = a.z-b.z;
    return std::sqrt(dx*dx+dy*dy+dz*dz);
  }

  inline double dist(geometry_msgs::Point b, PtXYZ a)
  {
    return dist(a,b);
  }


  inline double sqrDist(Eigen::Vector3f& a, PtXYZ& b)
  {
    const double dx = a[0]-b.x;
    const double dy = a[1]-b.y;
    const double dz = a[2]-b.z;
    return dx*dx+dy*dy+dz*dz;
  }

  inline double sqrDist(Eigen::Vector4f& a, Eigen::Vector4f& b)
  {
    const double dx = a[0]-b[0];
    const double dy = a[1]-b[1];
    const double dz = a[2]-b[2];
    return dx*dx+dy*dy+dz*dz;
  }

  inline double sqrDist(PtXYZ a, PtXYZ b)
  {
    const double dx = a.x-b.x;
    const double dy = a.y-b.y;
    const double dz = a.z-b.z;
    return dx*dx+dy*dy+dz*dz;
  }

  inline double sqrDist(PtXYZRGB a , PtXYZRGB b)
  {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return dx*dx + dy*dy + dz*dz;
  }

  inline double sqrDist(PtXYZ a, Eigen::Vector4f b)
  {
    const double dx = a.x-b[0];
    const double dy = a.y-b[1];
    const double dz = a.z-b[2];
    return dx*dx+dy*dy+dz*dz;
  }

  inline double sqrDistXY(PtXYZ a, PtXYZ b)
  {
    const double dx = a.x-b.x;
    const double dy = a.y-b.y;
    return dx*dx+dy*dy;
  }

  bool lineSegmentIntersection2D(pcl::PointXYZ a1, pcl::PointXYZ a2, pcl::PointXYZ b1, pcl::PointXYZ b2,
                                 pcl::PointXYZ& intersection);

  bool lineLineIntersection2D(pcl::PointXYZ a1, pcl::PointXYZ a2, pcl::PointXYZ b1, pcl::PointXYZ b2,
                              pcl::PointXYZ& intersection);

  bool pointIsBetweenOthers(pcl::PointXYZ& pt, pcl::PointXYZ& x1, pcl::PointXYZ& x2);

  double pointLineDistance2D(pcl::PointXYZ& pt, pcl::PointXYZ& a, pcl::PointXYZ& b);

  inline double colorDist(PtXYZRGB pt1, PtXYZRGB pt2)
  {
    /* pt1,pt2 - points in cloud
       ---
       returns L2 distance */
    return static_cast<double>( sqrt( (pt1.r - pt2.r) * (pt1.r - pt2.r) +
                                      (pt1.g - pt2.g) * (pt1.g - pt2.g) +
                                      (pt1.b - pt2.b) * (pt1.b - pt2.b) ) );
  }
  void writeCloud(PCXYZRGB::Ptr cloud,
                  std::string file);
  void writeCloud(PCXYZRGB::Ptr cloud,
                  std::string file,
                  int i);
  /**
   * Naively determine if two point clouds intersect based on distance threshold
   * between points.
   *
   * @param cloud0 First cloud for interesection test
   * @param cloud1 Second cloud for interesection test
   *
   * @return true if any points from cloud0 and cloud1 have distance less than
   * voxel_down_res_
   */

  bool cloudsIntersect(PCXYZ::Ptr cloud0, PCXYZ::Ptr cloud1,
                       double thresh);

  bool pointIntersectsCloud(PCXYZ::Ptr cloud, geometry_msgs::Point pt,
                            double thresh);

  float pointLineXYDist(PtXYZ p,Eigen::Vector3f vec,Eigen::Vector4f base);


  Eigen::Vector4f splitPlaneVertical(Eigen::Vector3f l_pt, Eigen::Vector3f l_dir);

  void splitCloud3D(Eigen::Vector4f& hessian, PCXYZ::Ptr to_split, PCXYZ::Ptr c0,
                    PCXYZ::Ptr c1);

  void lineCloudIntersection(PCXYZ::Ptr cloud, Eigen::Vector3f vec,
                             Eigen::Vector4f base, PCXYZ::Ptr line_cloud,
                             double thresh);

  void lineCloudIntersectionEndPoints(PCXYZ::Ptr cloud, Eigen::Vector3f vec,
                                      Eigen::Vector4f base, std::vector<PtXYZ>& end_points,
                                      double thresh);

  /**
   * Find the line intersecting two points
   *
   * @param p1 Plane 1 in Hessian normal form
   * @param p2 Plane 2 in Hessian normal form
   * @param l_pt Point on the intersecting line (returned)
   * @param l_dir Vector definining direction of the line (returned)
   *
   * @return True if the line exists, false if the two planes are parallel
   */
  bool planePlaneIntersection(Eigen::Vector4f p1, Eigen::Vector4f p2,
                              Eigen::Vector3f l_pt, Eigen::Vector3f l_dir);

  void passThroughFilter(PCXYZ::Ptr pcl_in, PCXYZ::Ptr pcl_out,
                         std::string fieldName, double fieldMin, double fieldMax,
                         bool keep_organized=false);
  void passThroughFilter(PCXYZRGB::Ptr pcl_in, PCXYZRGB::Ptr pcl_out,
                         std::string fieldName, double fieldMin, double fieldMax,
                         bool keep_organized=false);

  /**
   * Downsample a cloud using a voxel grid. Leaf size is controlled by voxel_down_res_
   *
   * @param cloud_in The cloud to downsample
   * @param cloud_down the downsampled cloud
   * @param voxel_down_res resolution of voxel grid
   */

  void downsampleCloud(PCXYZRGB::Ptr cloud_in, PCXYZRGB::Ptr cloud_down, double voxel_down_res);
  void downsampleCloud(PCXYZ::Ptr cloud_in, PCXYZ::Ptr cloud_down, double voxel_down_res);

  /**    * Filter the cloud then downsample
   *
   * @param cloud_in   The cloud to filter and downsample
   * @param cloud_down [Returned] The downsampled cloud
   * @param voxel_down_res resolution of voxel grid
   * @param min_x      Min x for filter
   * @param max_x      Max x for filter
   * @param min_y      Min y for filter
   * @param max_y      Max y for filter
   * @param min_z      Min z for filter
   * @param max_z      Max z for filter
   * @param filter_y   If true filter in y direction, default to false
   */
  void downsampleFilterCloud(PCXYZ::Ptr cloud_in, PCXYZ::Ptr cloud_down,
                             double voxel_down_res,
                             double min_x, double max_x,
                             double min_y, double max_y,
                             double min_z, double max_z, bool filter_y = false);




  /**
   * Method to project the current proto objects into an image
   *
   * @param objs The set of objects
   * @param img_in An image of correct size for the projection
   * @param projMat projectin matrix to the camera frame
   *
   * @return Image containing the projected objects
   */
  cv::Mat projectProtoObjectsIntoImage(ProtoObjects& objs, cv::Size img_size, ProjMat projMat);

  /**
   * Method to project the current proto object into an image
   *
   * @param obj The object
   * @param img_in An image of correct size for the projection
   * @param projMat projectin matrix to the camera frame
   *
   * @return Image containing the projected object
   */
  cv::Mat projectProtoObjectIntoImage(ProtoObject& obj, cv::Size img_size, ProjMat projMat);

  void projectPointCloudIntoImage(PCXYZ::Ptr cloud, cv::Mat& lbl_img, ProjMat projMat, int label);
  Eigen::Vector3d projectPointIntoImage(PtXYZ pt, ProjMat projMat);

  void findNormals(PCXYZRGB::Ptr in_pcl, PCNormal::Ptr normals);

  void findCloudBoundingBoxPCA(PCXYZRGB::Ptr&,geometry_msgs::Pose&,double&,double&,double&,bool);
  }
}

#endif
