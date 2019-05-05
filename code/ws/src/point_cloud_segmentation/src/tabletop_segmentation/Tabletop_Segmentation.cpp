#include <Tabletop_Segmentation.h>


namespace PCSeg
{
  namespace tabletop
  {
  TabletopSegmentation::TabletopSegmentation(ros::NodeHandle n):
    n_(n),
    raw_cloud_(new PCXYZRGB),
    old_raw_cloud_(new PCXYZRGB),
    loop_rate_(10.0)
  {
    // read in parameters
    double tmp_rate;
    ros::NodeHandle n_private("~");
    // basics
    n_private.param("Loop_Rate",tmp_rate, 10.0);
    n_private.param("debug",debug_,true);
    // tabletop filtering
    n_private.param("max_z", max_z_, 2.0);
    n_private.param("min_z", min_z_, 0.0);
    n_private.param("filter_bias",filter_bias_,0.02);
    // ransac table top
    n_private.param("ransac_max_iterations",ransac_max_iter_,100);
    n_private.param("RANSAC_Distance_Threshold",ransac_dist_thresh_,0.015);
    n_private.param("depth_scale",dpt_scale_,1.0);
    // cloud subtraction
    n_private.param("Max_Point_Distance",max_motion_,0.01);
    n_private.param("Max_Color_Distance",max_color_,30.0);
    n_private.param("Morph_Size",morph_size_,2);
    n_private.param("Minimum_Number_Points_4_Motion",min_motion_size_,1000);
    // read in decision varaibles
    n_private.param("Send_Motion_Cloud",send_motion_,true);
    n_private.param("Send_Object_Masks",send_object_masks_,false);
    n_private.param("Send_Object_Bounding_Box",send_object_bb_,false);
    n_private.param("keep_organized",keep_organized_,false);
    n_private.param("find_table_plane_once",find_one_plane_,false);
    n_private.param("Send_Table",send_table_,false);
    n_private.param("Send_Background",send_bg_,false);
    n_private.param("Use_Srvs",use_srv_,false);
    n_private.param("Use_Topics",use_topics_,false);
    n_private.param("Send_Color_Images",send_color_imgs_,false);
    n_private.param("Send_Raw_Data",send_raw_data_,false);
    
    // set up color vector
    getColorVec();

    cloud_sub_ = n_.subscribe("/cloud",1,&TabletopSegmentation::cloudCB,this);

    // set the loop rate
    loop_rate_ = ros::Rate(tmp_rate);
    init_ = true;
    got_data_ = false;
  }

  void TabletopSegmentation::cloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    if(send_motion_)
      raw_cloud_.swap(old_raw_cloud_);
    sensor_msgs::PointCloud2 tmp_cloud = *cloud_msg;
    pcl::moveFromROSMsg(tmp_cloud,*raw_cloud_);
    got_data_ = true; 
  }

  void TabletopSegmentation::findPlane(PCXYZRGB::Ptr in_pcl,
                                       pcl::PointIndices::Ptr inliers,
                                       pcl::ModelCoefficients::Ptr coefficients)
  {
    /* using RANSAC this will find the table plane */
    pcl::SACSegmentation<PtXYZRGB> seg;
    seg.setMaxIterations(ransac_max_iter_);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ransac_dist_thresh_);
    seg.setInputCloud(in_pcl);
    seg.segment(*inliers,*coefficients);
  }
  void TabletopSegmentation::filterPlane(PCXYZRGB::Ptr in_cloud,PCXYZRGB::Ptr out_cloud)
  {
    double x,y,z;
    out_cloud->clear();
    out_cloud->height = 1;
    for(int i=0; i < in_cloud->points.size(); ++i)
    {
      if( PCSeg::utils::notNaN(in_cloud->points[i]) )
      {
        x = in_cloud->points[i].x;
        y = in_cloud->points[i].y;
        z = in_cloud->points[i].z;
        if( a_ * x + b_ * y + c_ * z + d_ + filter_bias_ > .000001 && 
            z < max_z_ && z > min_z_)
        {
          out_cloud->points.push_back(in_cloud->points[i]);
          out_cloud->width+=1;
        }
      }
    }
  }

  void TabletopSegmentation::filterPlane(PCXYZRGB::Ptr in_cloud, PCNormal::Ptr in_nrms, 
                                         PCXY::Ptr in_uv,
                                         PCXYZRGB::Ptr out_cloud, PCNormal::Ptr out_nrms, 
                                         PCXY::Ptr out_uv)
  {
    double x,y,z;
    out_cloud->clear();
    out_cloud->height = 1;
    out_nrms->clear();
    out_nrms->height = 1;
    out_uv->clear();
    out_uv->height = 1;
    for(int i=0; i < in_cloud->points.size(); ++i)
    {
      if( PCSeg::utils::notNaN(in_cloud->points[i]) && PCSeg::utils::notNaN(in_nrms->points[i]) )
      {
        x = in_cloud->points[i].x;
        y = in_cloud->points[i].y;
        z = in_cloud->points[i].z;
        if( a_ * x + b_ * y + c_ * z + d_ + filter_bias_ > .000001 && 
            z < max_z_ && z > min_z_)
        {
          out_cloud->points.push_back(in_cloud->points[i]);
          out_nrms->points.push_back(in_nrms->points[i]);
          out_uv->points.push_back(in_uv->points[i]);
          out_cloud->width+=1;
          out_nrms->width+=1;
          out_uv->width+=1;
        }
      }
    }
  }
  
  void TabletopSegmentation::filterPlaneMotion(PCXYZRGB::Ptr in_cloud,
                                               PCNormal::Ptr in_nrm,
                                               PCXY::Ptr in_uv,
                                               cv::Mat motion_mat,
                                               PCXYZRGB::Ptr motion_cloud,
                                               PCNormal::Ptr motion_nrm, 
                                               PCXY::Ptr motion_uv,
                                               PCXYZRGB::Ptr objs_cloud,  
                                               PCNormal::Ptr objs_nrm, 
                                               PCXY::Ptr objs_uv)
  {
    double x,y,z;
    int u,v;
    // set up headers
    objs_cloud->header = in_cloud->header;
    objs_nrm->header = in_cloud->header;
    objs_uv->header = in_cloud->header;

    motion_cloud->header = in_cloud->header;
    motion_nrm->header   = in_cloud->header;
    motion_uv->header    = in_cloud->header;

    objs_cloud->clear();
    objs_cloud->height = 1;
    objs_nrm->clear();
    objs_nrm->height = 1;
    objs_uv->clear();
    objs_uv->height = 1;

    motion_cloud->clear();
    motion_cloud->height = 1;
    motion_nrm->clear();
    motion_nrm->height = 1;
    motion_uv->clear();
    motion_uv->height = 1;
    for(int i=0; i < in_cloud->points.size(); ++i)
    {
      if( PCSeg::utils::notNaN(in_cloud->points[i]) && PCSeg::utils::notNaN(in_nrm->points[i]) ) 
        // check it is real
      {
        u = in_uv->points[i].x;
        v = in_uv->points[i].y;
        x = in_cloud->points[i].x;
        y = in_cloud->points[i].y;
        z = in_cloud->points[i].z;
        if( a_ * x + b_ * y + c_ * z + d_ + filter_bias_ > .000001 && 
            z < max_z_ && z > min_z_) // on table
        {
          objs_cloud->points.push_back(in_cloud->points[i]);
          objs_nrm->points.push_back(in_nrm->points[i]);
          objs_uv->points.push_back(in_uv->points[i]);
          objs_cloud->width+=1;
          objs_nrm->width+=1;
          objs_uv->width+=1;
          if( motion_mat.at<char>(v,u) != 0 ) // moved
          {
            motion_cloud->points.push_back(in_cloud->points[i]);
            motion_nrm->points.push_back(in_nrm->points[i]);
            motion_uv->points.push_back(in_uv->points[i]);
            motion_cloud->width+=1;
            motion_nrm->width+=1;
            motion_uv->width+=1;
          }
        }
      }
    }
  }
  void TabletopSegmentation::filterPlaneMotionKeepTableBG(PCXYZRGB::Ptr in_cloud,
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
                                                          PCXY::Ptr bg_uv)
  {
    double x,y,z;
    int u,v;
    // set up headers
    objs_cloud->header = in_cloud->header;
    objs_nrm->header = in_cloud->header;
    objs_uv->header = in_cloud->header;

    motion_cloud->header = in_cloud->header;
    motion_nrm->header   = in_cloud->header;
    motion_uv->header    = in_cloud->header;

    table_cloud->header = in_cloud->header;
    table_nrm->header = in_cloud->header;
    table_uv->header = in_cloud->header;

    bg_cloud->header = in_cloud->header;
    bg_nrm->header = in_cloud->header;
    bg_uv->header = in_cloud->header;

    objs_cloud->clear();
    objs_cloud->height = 1;
    objs_nrm->clear();
    objs_nrm->height = 1;
    objs_uv->clear();
    objs_uv->height = 1;

    motion_cloud->clear();
    motion_cloud->height = 1;
    motion_nrm->clear();
    motion_nrm->height = 1;
    motion_uv->clear();
    motion_uv->height = 1;
    
    table_cloud->clear();
    table_cloud->height = 1;
    table_nrm->clear();
    table_nrm->height = 1;
    table_uv->clear();
    table_uv->height = 1;

    bg_cloud->clear();
    bg_cloud->height = 1;
    bg_nrm->clear();
    bg_nrm->height = 1;
    bg_uv->clear();
    bg_uv->height = 1;

    double score =0;
    for(int i=0; i < in_cloud->points.size(); ++i)
    {
      if( PCSeg::utils::notNaN(in_cloud->points[i]) && PCSeg::utils::notNaN(in_nrm->points[i]) ) 
        // check it is real
      {
        u = in_uv->points[i].x;
        v = in_uv->points[i].y;
        x = in_cloud->points[i].x;
        y = in_cloud->points[i].y;
        z = in_cloud->points[i].z;
        score = a_ * x + b_ * y + c_ * z + d_;
        if( score + filter_bias_ > .000001 && 
            z < max_z_ && z > min_z_) // on table
        {
          objs_cloud->points.push_back(in_cloud->points[i]);
          objs_nrm->points.push_back(in_nrm->points[i]);
          objs_uv->points.push_back(in_uv->points[i]);
          objs_cloud->width+=1;
          objs_nrm->width+=1;
          objs_uv->width+=1;
          if( motion_mat.at<char>(v,u) != 0 ) // moved
          {
            motion_cloud->points.push_back(in_cloud->points[i]);
            motion_nrm->points.push_back(in_nrm->points[i]);
            motion_uv->points.push_back(in_uv->points[i]);
            motion_cloud->width+=1;
            motion_nrm->width+=1;
            motion_uv->width+=1;
          }
        }
        else if( fabs(score) < ransac_dist_thresh_ && z < max_z_ && z > min_z_)
        {
          table_cloud->points.push_back(in_cloud->points[i]);
          table_nrm->points.push_back(in_nrm->points[i]);
          table_uv->points.push_back(in_uv->points[i]);
          table_cloud->width+=1;
          table_nrm->width+=1;
          table_uv->width+=1;
        }
        else
        {
          bg_cloud->points.push_back(in_cloud->points[i]);
          bg_nrm->points.push_back(in_nrm->points[i]);
          bg_uv->points.push_back(in_uv->points[i]);
          bg_cloud->width+=1;
          bg_nrm->width+=1;
          bg_uv->width+=1;
        } 
      }
    }
  }

  void TabletopSegmentation::filterPlaneKeepTableBG(PCXYZRGB::Ptr in_cloud, PCNormal::Ptr in_nrm, 
                                                    PCXY::Ptr in_uv,
                                                    PCXYZRGB::Ptr objs_cloud, PCNormal::Ptr objs_nrm,
                                                    PCXY::Ptr objs_uv,
                                                    PCXYZRGB::Ptr table_cloud, 
                                                    PCNormal::Ptr table_nrm, PCXY::Ptr table_uv,
                                                    PCXYZRGB::Ptr bg_cloud, PCNormal::Ptr bg_nrm, 
                                                    PCXY::Ptr bg_uv)
  {
    double x,y,z;
    // set up headers
    objs_cloud->header = in_cloud->header;
    objs_nrm->header = in_cloud->header;
    objs_uv->header = in_cloud->header;

    table_cloud->header = in_cloud->header;
    table_nrm->header = in_cloud->header;
    table_uv->header = in_cloud->header;

    bg_cloud->header = in_cloud->header;
    bg_nrm->header = in_cloud->header;
    bg_uv->header = in_cloud->header;

    objs_cloud->clear();
    objs_cloud->height = 1;
    objs_nrm->clear();
    objs_nrm->height = 1;
    objs_uv->clear();
    objs_uv->height = 1;
    
    table_cloud->clear();
    table_cloud->height = 1;
    table_nrm->clear();
    table_nrm->height = 1;
    table_uv->clear();
    table_uv->height = 1;

    bg_cloud->clear();
    bg_cloud->height = 1;
    bg_nrm->clear();
    bg_nrm->height = 1;
    bg_uv->clear();
    bg_uv->height = 1;

    double score = 0.0;
    for(int i=0; i < in_cloud->points.size(); ++i)
    {
      if( PCSeg::utils::notNaN(in_cloud->points[i]) && PCSeg::utils::notNaN(in_nrm->points[i]) ) 
        // check it is real
      {
        x = in_cloud->points[i].x;
        y = in_cloud->points[i].y;
        z = in_cloud->points[i].z;
        score = a_ * x + b_ * y + c_ * z + d_;
        if( score + filter_bias_ > .000001 && 
            z < max_z_ && z > min_z_) // on table
        {
          objs_cloud->points.push_back(in_cloud->points[i]);
          objs_nrm->points.push_back(in_nrm->points[i]);
          objs_uv->points.push_back(in_uv->points[i]);
          objs_cloud->width+=1;
          objs_nrm->width+=1;
          objs_uv->width+=1;
        }
        else if( fabs(score) < ransac_dist_thresh_)
        {
          table_cloud->points.push_back(in_cloud->points[i]);
          table_nrm->points.push_back(in_nrm->points[i]);
          table_uv->points.push_back(in_uv->points[i]);
          table_cloud->width+=1;
          table_nrm->width+=1;
          table_uv->width+=1;
        }
        else
        {
          bg_cloud->points.push_back(in_cloud->points[i]);
          bg_nrm->points.push_back(in_nrm->points[i]);
          bg_uv->points.push_back(in_uv->points[i]);
          bg_cloud->width+=1;
          bg_nrm->width+=1;
          bg_uv->width+=1;
        } 
      }
    }
  }

  void TabletopSegmentation::subtractCloud(PCXYZRGB::Ptr cloud1, PCXYZRGB::Ptr cloud2,
                                           cv::Mat& motion_mask)
  {
    /* cloud1      - first cloud (in time)
       cloud2      - second cloud ( in time)
       motion_mask - mask with 0 if still and 1 otherwise
       ---
       Subtractions cloud1 from 2 and generates a mask of image */
    float dist, color_dist;
    PtXYZRGB pt1,pt2;
    cv::Mat motion_temp(cloud1->height,cloud1->width,CV_8UC1,cv::Scalar(0));
    for(int u=0; u < cloud1->width; ++u)
    {
      for(int v=0; v < cloud2->height; ++v)
      {
        pt1 = cloud1->at(u,v);
        pt2 = cloud2->at(u,v);
        if( PCSeg::utils::dist(pt1,pt2) < max_motion_ && PCSeg::utils::colorDist(pt1,pt2) < max_color_)
          motion_temp.at<char>(v,u) = 0;
        else
        motion_temp.at<char>(v,u) = 255;
      }
    }
    // post process motion mat
    // run open operation
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT, 
                                                 cv::Size( 2*morph_size_ + 1, 2*morph_size_+1 ), 
                                                 cv::Point( morph_size_, morph_size_ ) );
    cv::morphologyEx(motion_temp,motion_mask,cv::MORPH_CLOSE,element);
    
  }


  void TabletopSegmentation::getColorVec()
  {
    /* Generates a list of colors that are most different from all 
       previous colors and respects color blind people */
    colors_.resize(28);
    colors_[0]  = cv::Vec3b(0,0,0);
    colors_[1]  = cv::Vec3b(255,255,255);
    colors_[2]  = cv::Vec3b(240,163,255);
    colors_[3]  = cv::Vec3b(0,117,220);
    colors_[4]  = cv::Vec3b(153,63,0);
    colors_[5]  = cv::Vec3b(76,0,92);
    colors_[6]  = cv::Vec3b(25,25,25);
    colors_[7]  = cv::Vec3b(0,92,49);
    colors_[8]  = cv::Vec3b(43,206,72);
    colors_[9]  = cv::Vec3b(255,204,153);
    colors_[10] = cv::Vec3b(128,128,128);
    colors_[11] = cv::Vec3b(148,255,181);
    colors_[12] = cv::Vec3b(143,124,0);
    colors_[13] = cv::Vec3b(157,204,0);
    colors_[14] = cv::Vec3b(194,0,136);
    colors_[15] = cv::Vec3b(0,51,128);
    colors_[16] = cv::Vec3b(255,164,5);
    colors_[17] = cv::Vec3b(255,168,187);
    colors_[18] = cv::Vec3b(66,102,0);
    colors_[19] = cv::Vec3b(255,0,16);
    colors_[20] = cv::Vec3b(94,241,242);
    colors_[21] = cv::Vec3b(0,153,143);
    colors_[22] = cv::Vec3b(224,255,102);
    colors_[23] = cv::Vec3b(116,10,255);
    colors_[24] = cv::Vec3b(153,0,0);
    colors_[25] = cv::Vec3b(255,255,128);
    colors_[26] = cv::Vec3b(255,255,0);
    colors_[27] = cv::Vec3b(255,80,5);
  }
  }
}
