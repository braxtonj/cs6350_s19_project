#include <multiObjectSegmentation.h>

namespace PCSeg
{
  namespace tabletop
  { 
  
  MultiObjectSegmentation::MultiObjectSegmentation(ros::NodeHandle n):TabletopSegmentation(n)
  {
    // debug intit stuff goes here 
    // set up srv and publishers
    // define the publishers
    if(use_topics_)
      proc_pub_ = n_.advertise<filterMsg>("Multi_Object_Filtered",1);
    
    if(debug_)
    {
      ROS_INFO("Initialized Multiobject Tabletop Segmentation in debug mode");
    }
    else
      ROS_INFO("Data Processer Initialized");
  }
  void MultiObjectSegmentation::spin()
  {
    ROS_INFO("Called Spin Function");
    if(send_motion_)
      motionSpin();
    else
      basicSpin();
  }

  void MultiObjectSegmentation::basicSpin()
  {
    ROS_INFO("Comming soon to a computer near you");


    filterMsg filter_msg;
    PCNormal::Ptr nrm_cloud(new PCNormal);
    PCXY::Ptr uv_cloud(new PCXY);
    
    PCXYZRGB::Ptr objs_cloud(new PCXYZRGB);
    PCNormal::Ptr objs_nrm(new PCNormal);
    PCXY::Ptr     objs_uv(new PCXY);


    PCXYZRGB::Ptr table_cloud(new PCXYZRGB);
    PCNormal::Ptr table_nrm(new PCNormal);
    PCXY::Ptr     table_uv(new PCXY);

    PCXYZRGB::Ptr bg_cloud(new PCXYZRGB);
    PCNormal::Ptr bg_nrm(new PCNormal);
    PCXY::Ptr     bg_uv(new PCXY);

    pcl::PointIndices::Ptr table_indices (new pcl::PointIndices );
    pcl::ModelCoefficients::Ptr table_params( new pcl::ModelCoefficients);
    
    if (debug_)
      ROS_INFO("Initialized Data Structure");
    while(n_.ok())
    {
      loop_rate_.sleep();
      ros::spinOnce();
      
      if(!got_data_)
        continue;
      got_data_ = false;

      
      // fill uv_cloud
      if(init_)
      {
        // resize to cloud sizes if initial scene
        uv_cloud->resize(raw_cloud_->width * raw_cloud_->height);
        uv_cloud->width = raw_cloud_->width;
        uv_cloud->height = raw_cloud_->height;
        uv_cloud->is_dense = true;
        
        // fill uv cloud (note this is constant
        for(int u =0; u < raw_cloud_->width; ++u)
        {
          for(int v =0; v < raw_cloud_->height; ++v)
          {
            uv_cloud->at(u,v).x = u;
            uv_cloud->at(u,v).y = v;
          }
        }
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
      
      // find normals
      PCSeg::utils::findNormals(raw_cloud_,nrm_cloud);
      
      if(send_table_ || send_bg_)
      {
        //filterPlane but keep bg and table
        filterPlaneKeepTableBG(raw_cloud_,
                               nrm_cloud,
                               uv_cloud,
                               //outputs
                               objs_cloud,
                               objs_nrm,
                               objs_uv,
                               table_cloud,
                               table_nrm,
                               table_uv,
                               bg_cloud,
                               bg_nrm,
                               bg_uv);
      }
      else
      {
        // filter out background and table
        ROS_INFO("About to filter plane");
        filterPlane(raw_cloud_,
                    nrm_cloud,
                    uv_cloud,
                    // outputs
                    objs_cloud,
                    objs_nrm,
                    objs_uv);
        if(debug_)
          ROS_INFO("Filtered out the background and table");
      }
      
      // publish the message
      if(send_raw_data_)
      {
        // send raw data
        pcl::toROSMsg(*raw_cloud_,filter_msg.cloud);
        pcl::toROSMsg(*nrm_cloud,filter_msg.nrm_cloud);
      }

      // send object data
      pcl::toROSMsg(*objs_cloud,filter_msg.objs_cloud);
      pcl::toROSMsg(*objs_nrm,filter_msg.objs_nrm);
      pcl::toROSMsg(*objs_uv, filter_msg.objs_uv);
      
      if( send_table_)
      {
        pcl::toROSMsg(*table_cloud,filter_msg.table_cloud);
        pcl::toROSMsg(*table_nrm,filter_msg.table_nrm);
        pcl::toROSMsg(*table_uv, filter_msg.table_uv);
      }
      if( send_bg_ )
      {
        pcl::toROSMsg(*bg_cloud,filter_msg.bg_cloud);
        pcl::toROSMsg(*bg_nrm,filter_msg.bg_nrm);
        pcl::toROSMsg(*bg_uv, filter_msg.bg_uv);
      }
      if(debug_)
        ROS_INFO("\tPacked data");
      
      proc_pub_.publish(filter_msg);
      
      if(debug_)
        ROS_INFO("\tData Sent");
    }
    
  }
  void MultiObjectSegmentation::motionSpin()
  {
    // define data structures
    filterMsg filter_msg;
    PCNormal::Ptr nrm_cloud(new PCNormal);
    PCXY::Ptr uv_cloud(new PCXY);

    PCXYZRGB::Ptr objs_cloud(new PCXYZRGB);
    PCNormal::Ptr objs_nrm(new PCNormal);
    PCXY::Ptr objs_uv(new PCXY);

    PCXYZRGB::Ptr motion_cloud(new PCXYZRGB);
    PCNormal::Ptr motion_nrm(new PCNormal);
    PCXY::Ptr motion_uv(new PCXY);
    cv::Mat motion_mat;


    PCXYZRGB::Ptr table_cloud(new PCXYZRGB);
    PCNormal::Ptr table_nrm(new PCNormal);
    PCXY::Ptr     table_uv(new PCXY);

    PCXYZRGB::Ptr bg_cloud(new PCXYZRGB);
    PCNormal::Ptr bg_nrm(new PCNormal);
    PCXY::Ptr     bg_uv(new PCXY);

    pcl::PointIndices::Ptr table_indices (new pcl::PointIndices );
    pcl::ModelCoefficients::Ptr table_params (new pcl::ModelCoefficients);

    if(debug_)
      ROS_INFO("Initialized Data");
    while(n_.ok())
    {
      loop_rate_.sleep();
      ros::spinOnce();

      // check for new data  
      if(!got_data_)
      {
        continue;
      }

      if(debug_)
        ROS_INFO("Got data");
      // reset got data
      got_data_ = false;

      // fill uv_cloud
      if(init_)
      {
        // resize to cloud sizes if initial scene
        uv_cloud->resize(raw_cloud_->width * raw_cloud_->height);
        uv_cloud->width = raw_cloud_->width;
        uv_cloud->height = raw_cloud_->height;
        uv_cloud->is_dense = true;
        // set up motion mat
        motion_mat.create(raw_cloud_->height,raw_cloud_->width,CV_8UC1);

        // fill uv cloud (note this is constant
        for(int u =0; u < raw_cloud_->width; ++u)
        {
          for(int v =0; v < raw_cloud_->height; ++v)
          {
            uv_cloud->at(u,v).x = u;
            uv_cloud->at(u,v).y = v;
          }
        }
        if( send_table_ )
          filter_msg.table_params.resize(4);
      }
      
      if(init_ || !find_one_plane_) // note if find_one_plane_ is true this will only run once
      {
        // use ransac to find the table plane
        findPlane(raw_cloud_,table_indices,table_params);
        // run RANSAC saving inlier indices to table_indices 
        a_ = table_params->values[0];
        b_ = table_params->values[1];
        c_ = table_params->values[2];
        d_ = table_params->values[3];
        if(send_table_)
        {
          filter_msg.table_params[0] = a_;
          filter_msg.table_params[1] = b_;
          filter_msg.table_params[2] = c_;
          filter_msg.table_params[3] = d_;
        }
        if(debug_)
          ROS_INFO_STREAM("\tFound table: a=" << a_ << " b=" << b_ << " c=" << c_ << " d=" << d_);
      }

      if(init_)
      {
        // pass on first data to start getting diff
        init_ = false;
        if(debug_)
          ROS_INFO("\tPassing first data point");
      }
      else // now we are running
      {
        // find normals
        PCSeg::utils::findNormals(raw_cloud_,nrm_cloud);

        // subtract cloud
        subtractCloud(old_raw_cloud_, raw_cloud_, motion_mat);

        if(debug_)
          ROS_INFO("\tSubtracted clouds");

        if(send_table_ || send_bg_)
        {
          // new record for longest function call :)
          filterPlaneMotionKeepTableBG(raw_cloud_,
                                       nrm_cloud,
                                       uv_cloud,
                                       motion_mat,
                                       motion_cloud,
                                       motion_nrm, 
                                       motion_uv,
                                       objs_cloud,  
                                       objs_nrm, 
                                       objs_uv,
                                       table_cloud, 
                                       table_nrm, 
                                       table_uv,
                                       bg_cloud, 
                                       bg_nrm, 
                                       bg_uv);
        }
        else
        {
          // filter out background and table
          filterPlaneMotion(raw_cloud_,
                            nrm_cloud,
                            uv_cloud,
                            motion_mat,
                            // outputs
                            motion_cloud,
                            motion_nrm,
                            motion_uv,
                            objs_cloud,
                            objs_nrm,
                            objs_uv);
          if(debug_)
            ROS_INFO("\tFiltered Plane w/Motion");
        }

        // publish the message
        if(send_raw_data_)
        {
          // send raw data
          pcl::toROSMsg(*raw_cloud_,filter_msg.cloud);
          pcl::toROSMsg(*nrm_cloud,filter_msg.nrm_cloud);
        }

        filter_msg.motion_mask = *cv_bridge::CvImage(std_msgs::Header(),
                                                     "8UC1",motion_mat).toImageMsg();
        // send object data
        pcl::toROSMsg(*objs_cloud,filter_msg.objs_cloud);
        pcl::toROSMsg(*objs_nrm,filter_msg.objs_nrm);
        pcl::toROSMsg(*objs_uv, filter_msg.objs_uv);

        // send motion data
        pcl::toROSMsg(*motion_cloud,filter_msg.objs_motion_cloud);
        pcl::toROSMsg(*motion_nrm,filter_msg.objs_motion_nrm);
        pcl::toROSMsg(*motion_uv, filter_msg.objs_motion_uv);
      
        if( send_table_)
        {
          pcl::toROSMsg(*table_cloud,filter_msg.table_cloud);
          pcl::toROSMsg(*table_nrm,filter_msg.table_nrm);
          pcl::toROSMsg(*table_uv, filter_msg.table_uv);
        }
        if( send_bg_ )
        {
          pcl::toROSMsg(*bg_cloud,filter_msg.bg_cloud);
          pcl::toROSMsg(*bg_nrm,filter_msg.bg_nrm);
          pcl::toROSMsg(*bg_uv, filter_msg.bg_uv);
        }

        if(debug_)
          ROS_INFO("\tPacked data");

        proc_pub_.publish(filter_msg);

        if(debug_)
          ROS_INFO("\tData Sent");
      }
    }
    
  }

  }
}
