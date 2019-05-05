#include <singleObjectSegmentation.h>

#include <point_cloud_segmentation/drawEllipsoid.h>
typedef sensor_msgs::PointCloud2 MsgCloud;
namespace PCSeg
{
  namespace tabletop
  {
  SingleObjectSegmentation::SingleObjectSegmentation(ros::NodeHandle n):
    TabletopSegmentation(n)
  {
    /* class constructor */
  }

  void SingleObjectSegmentation::spin()
  {
    cv::Mat rgb_img;
    PCXYZRGB::Ptr obj_cloud(new PCXYZRGB);
    
    PCXYZRGB::Ptr table_cloud(new PCXYZRGB);

    PCXYZRGB::Ptr bg_cloud(new PCXYZRGB);

    pcl::PointIndices::Ptr table_indices (new pcl::PointIndices );
    pcl::ModelCoefficients::Ptr table_params( new pcl::ModelCoefficients);
    while(n_.ok())
    {
      loop_rate_.sleep();
      ros::spinOnce();

      if(!got_data_)
        continue;
      got_data_ = false;

      if(init_)
      {
        rgb_img.create(raw_cloud_->height, raw_cloud_->width, CV_8UC3);
      }

      if(!find_one_plane_ || init_) // note if find_one_plane_ is true this will only run once
      {
        // use ransac to find the table plane
        findPlane(raw_cloud_,table_indices,table_params);
        // run RANSAC saving inlier indices to table_indices 
        a_ = table_params->values[0];
        b_ = table_params->values[1];
        c_ = table_params->values[2];
        d_ = table_params->values[3];
        if(debug_)
          ROS_INFO_STREAM("\tFound table: a=" << a_ << " b=" << b_ << " c=" << c_ << " d=" << d_);
        init_ = false;
      }

      PtXYZRGB pt;
      for(int u=0; u < raw_cloud_->width; ++u)
      {
        for(int v=0; v < raw_cloud_->height; ++v)
        {
          pt = raw_cloud_->at(u,v);
          rgb_img.at<cv::Vec3b>(v,u) = cv::Vec3b(pt.b,pt.g,pt.r);
        }
      }
      
      // get only the object remaining
      filterPlane(raw_cloud_,obj_cloud);

      // fit an ellipsoid to it
      geometry_msgs::Pose pose;
      double width;
      double height;
      double depth;

      utils::findCloudBoundingBoxPCA(obj_cloud,pose,width,height,depth,false);

      sensor_msgs::ImagePtr RGBmsg = cv_bridge::CvImage(std_msgs::Header(),"rgb8",rgb_img).toImageMsg();
      // have the ellipsoid draw on the rgb image.
      point_cloud_segmentation::drawEllipsoid::Request req;
      point_cloud_segmentation::drawEllipsoid::Response res;

      req.a = width;
      req.b = height;
      req.c = depth;
      
      req.pose = pose;
      req.rgb_img = *RGBmsg;

      MsgCloud send_pcl;
      pcl::toROSMsg(*raw_cloud_,send_pcl);
      req.raw_cloud = send_pcl;

      ros::ServiceClient client = n_.serviceClient<point_cloud_segmentation::drawEllipsoid>("draw_ellipsoid");

      if(client.call(req,res))
      {
        ROS_INFO("Successfully Called");
      }
      
      
    }
  }

  }
}
