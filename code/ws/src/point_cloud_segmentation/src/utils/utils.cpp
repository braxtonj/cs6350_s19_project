

#include "utils.h"

namespace PCSeg
{

    namespace utils
    {

    bool cloudsintersect(PCXYZ::Ptr cloud0, PCXYZ::Ptr cloud1,
                         double thresh)
    {
      int moved_count = 0;
      for (unsigned int i = 0; i < cloud0->points.size(); ++i)
      {
        const PtXYZ pt0 = cloud0->points[i];
        for (unsigned int j = 0; j < cloud1->points.size(); ++j)
        {
          const PtXYZ pt1 = cloud1->points[i];
          if (dist(pt0, pt1) < thresh)
          {
            return true;
          }
        }
      }
      return false;
    }

    bool pointIntersectsCloud(PCXYZ::Ptr cloud, geometry_msgs::Point pt, double thresh)
    {
      for (unsigned int i = 0; i < cloud->size(); ++i)
      {
        const PtXYZ pt_c = cloud->at(i);
        if (dist(pt_c, pt) < thresh) return true;
      }
      return false;
    }

    bool lineSegmentIntersection2D(pcl::PointXYZ a1, pcl::PointXYZ a2, pcl::PointXYZ b1, pcl::PointXYZ b2,
                                   pcl::PointXYZ& intersection)
    {
      if (!lineLineIntersection2D(a1, a2, b1, b2, intersection))
      {
        return false;
      }
      // Test if intersection is between a1 and a2
      return (pointIsBetweenOthers(intersection, a1, a2) && pointIsBetweenOthers(intersection, b1, b2));
    }

    bool lineLineIntersection2D(pcl::PointXYZ a1, pcl::PointXYZ a2, pcl::PointXYZ b1, pcl::PointXYZ b2,
                                pcl::PointXYZ& intersection)
    {
      float denom = (a1.x-a2.x)*(b1.y-b2.y)-(a1.y-a2.y)*(b1.x-b2.x);
      if (denom == 0) // Parrallel lines
      {
        return false;
      }
      intersection.x = ((a1.x*a2.y - a1.y*a2.x)*(b1.x-b2.x) -
                        (a1.x - a2.x)*(b1.x*b2.y - b1.y*b2.x))/denom;
      intersection.y = ((a1.x*a2.y - a1.y*a2.x)*(b1.y-b2.y) -
                        (a1.y - a2.y)*(b1.x*b2.y - b1.y*b2.x))/denom;
      return true;
    }

    bool pointIsBetweenOthers(pcl::PointXYZ& pt, pcl::PointXYZ& x1, pcl::PointXYZ& x2)
    {
      return (pt.x >= std::min(x1.x, x2.x) && pt.x <= std::max(x1.x, x2.x) &&
              pt.y >= std::min(x1.y, x2.y) && pt.y <= std::max(x1.y, x2.y));
    }

    double pointLineDistance2D(pcl::PointXYZ& pt, pcl::PointXYZ& a, pcl::PointXYZ& b)
    {
      pcl::PointXYZ q(a.x - pt.x, a.y - pt.y, 0.0);
      pcl::PointXYZ n(b.x - a.x, b.y - a.y, 0.0);
      double norm_n = hypot(n.x, n.y);
      n.x /= norm_n;
      n.y /= norm_n;
      double q_dot_n = q.x*n.x+q.y*n.y;
      pcl::PointXYZ l(q.x - q_dot_n*n.x, q.y - q_dot_n*n.y, 0.0);
      return hypot(l.x, l.y);
    }

    float pointLineXYDist(PtXYZ p, Eigen::Vector3f vec, Eigen::Vector4f base)
    {
      Eigen::Vector3f x0(p.x,p.y,0.0);
      Eigen::Vector3f x1(base[0],base[1],0.0);
      Eigen::Vector3f x2 = x1+vec;
      Eigen::Vector3f num = (x0 - x1);
      num = num.cross(x0 - x2);
      Eigen::Vector3f den = x2 - x1;
      float d = num.norm()/den.norm();
      return d;
    }

    Eigen::Vector4f splitPlaneVertical(Eigen::Vector3f l_pt, Eigen::Vector3f l_dir)
    {
      const Eigen::Vector3f z_axis(0, 0, 1);
      Eigen::Vector3f n = l_dir.cross(z_axis);
      n = n/n.norm();
      float p = -(n[0]*l_pt[0]+n[1]*l_pt[1]+n[2]*l_pt[2]);
      Eigen::Vector4f hessian(n[0], n[1], n[2], p);
      return hessian;
    }

    void splitCloud3D(Eigen::Vector4f& hessian, PCXYZ::Ptr to_split, PCXYZ::Ptr c0,
                      PCXYZ::Ptr c1)
    {
      // Split the point clouds based on the half plane distance test
      pcl::PointIndices p1;
      for (unsigned int i = 0; i < to_split->size(); ++i)
      {
        const pcl::PointXYZ x = to_split->at(i);
        const float D = hessian[0]*x.x + hessian[1]*x.y + hessian[2]*x.z +
                        hessian[3];
        if (D > 0)
        {
          p1.indices.push_back(i);
        }
      }
      // Extract indices
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(to_split);
      extract.setIndices(boost::make_shared<pcl::PointIndices>(p1));
      extract.filter(*c0);
      extract.setNegative(true);
      extract.filter(*c1);
    }


    void lineCloudIntersection(PCXYZ::Ptr cloud, Eigen::Vector3f vec,
                               Eigen::Vector4f base, PCXYZ::Ptr line_cloud,
                               double thresh)
    {
      // Define parametric model of the line defined by base and vec and
      // test cloud memebers for distance from the line, if the distance is less
      // than epsilon say it intersects and add to the output set.
      pcl::PointIndices::Ptr line_inliers(new pcl::PointIndices);
      for (unsigned int i = 0; i < cloud->size(); ++i)
      {
        const PtXYZ pt = cloud->at(i);
        if (pointLineXYDist(pt, vec, base) < thresh)
        {
          line_inliers->indices.push_back(i);
        }
      }
      // Extract the interesecting points of the line.
      pcl::ExtractIndices<PtXYZ> extract;
      extract.setInputCloud(cloud);
      extract.setIndices(line_inliers);
      extract.filter(*line_cloud);
    }

    void lineCloudIntersectionEndPoints(PCXYZ::Ptr cloud, Eigen::Vector3f vec,
                                        Eigen::Vector4f base, std::vector<PtXYZ>& points,
                                        double thresh)
    {
      PCXYZ::Ptr intersection;
      lineCloudIntersection(cloud, vec, base, intersection, thresh);
      unsigned int min_y_idx = intersection->size();
      unsigned int max_y_idx = intersection->size();
      unsigned int min_x_idx = intersection->size();
      unsigned int max_x_idx = intersection->size();
      float min_y = 10000000.0f;
      float max_y = -100000000.0f;
      float min_x = min_y;
      float max_x = max_y;
      for (unsigned int i = 0; i < intersection->size(); ++i)
      {
        if (intersection->at(i).y < min_y)
        {
          min_y = intersection->at(i).y;
          min_y_idx = i;
        }
        if (intersection->at(i).y > max_y)
        {
          max_y = intersection->at(i).y;
          max_y_idx = i;
        }
        if (intersection->at(i).x < min_x)
        {
          min_x = intersection->at(i).x;
          min_x_idx = i;
        }
        if (intersection->at(i).x > max_x)
        {
          max_x = intersection->at(i).x;
          max_x_idx = i;
        }
      }
      const double y_dist_obs = max_y - min_y;
      const double x_dist_obs = max_x - min_x;
      int start_idx = min_x_idx;
      int end_idx = max_x_idx;

      if (x_dist_obs > y_dist_obs)
      {
        // Use X index
        if (vec[0] > 0)
        {
          // Use min
          start_idx = min_x_idx;
          end_idx = max_x_idx;
        }
        else
        {
          // use max
          start_idx = max_x_idx;
          end_idx = min_x_idx;
        }
      }
      else
      {
        // Use Y index
        if (vec[1] > 0)
        {
          // Use min
          start_idx = min_y_idx;
          end_idx = max_y_idx;
        }
        else
        {
          // use max
          start_idx = max_y_idx;
          end_idx = min_y_idx;
        }
      }
      PtXYZ start_point, end_point;
      start_point.x = intersection->at(start_idx).x;
      start_point.y = intersection->at(start_idx).y;
      start_point.z = intersection->at(start_idx).z;
      end_point.x = intersection->at(end_idx).x;
      end_point.y = intersection->at(end_idx).y;
      end_point.z = intersection->at(end_idx).z;
      points.push_back(start_point);
      points.push_back(end_point);
    }


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
                                Eigen::Vector3f l_pt, Eigen::Vector3f l_dir)
    {
      // Extract normals and constants from plane equations
      float n1_mag = sqrt(p1(0)*p1(0) + p1(1)*p1(1) + p1(2)*p1(2));
      Eigen::Vector3f n1;
      n1(0) = p1(0)/n1_mag;
      n1(1) = p1(1)/n1_mag;
      n1(2) = p1(2)/n1_mag;
      float d1 = p1(3)/n1_mag;

      float n2_mag = sqrt(p2(0)*p2(0) + p2(1)*p2(1) + p2(2)*p2(2));
      Eigen::Vector3f n2;
      n2(0) = p2(0)/n2_mag;
      n2(1) = p2(1)/n2_mag;
      n2(2) = p2(2)/n2_mag;
      float d2 = p2(3)/n2_mag;

      // Find direction of intersection
      l_dir = n1.cross(n2);

      // Test parallel planes
      if (n1(0) == n2(0) && n1(1) == n2(1) && n1(2) == n2(2) )
      {
        return false;
      }

      // Find point lying on both planes
      if (l_dir(3) > 0.0001)
      {
        // Assume z = 0
        l_pt(2) = 0.0;
        l_pt(0) = (-d1*n2(1) + d2*n1(1))/(n1(0)-n1(1)*n2(0));
        l_pt(1) = (-n2(0)*l_pt(0) - d2)/n2(1);
      }
      else
      {
        // Assume x = 0
        l_pt(0) = 0.0;
        l_pt(2) = (-d1*n2(1) + d2*n1(1))/(n1(2)-n1(1)*n2(2));
        l_pt(1) = (-n2(2)*l_pt(2) - d2)/n2(1);

      }

      return true;
    }


    void downsampleCloud(PCXYZRGB::Ptr cloud_in, PCXYZRGB::Ptr cloud_down, double voxel_down_res)
    {
      pcl::VoxelGrid<PtXYZRGB> downsample_outliers;
      downsample_outliers.setInputCloud(cloud_in);
      downsample_outliers.setLeafSize(voxel_down_res, voxel_down_res,
                                      voxel_down_res);
      downsample_outliers.filter(*cloud_down);
    }

    void downsampleCloud(PCXYZ::Ptr cloud_in, PCXYZ::Ptr cloud_down, double voxel_down_res)
    {
      pcl::VoxelGrid<PtXYZ> downsample_outliers;
      downsample_outliers.setInputCloud(cloud_in);
      downsample_outliers.setLeafSize(voxel_down_res, voxel_down_res,
                                      voxel_down_res);
      downsample_outliers.filter(*cloud_down);
    }

    void downSampleFilterCloud(PCXYZ::Ptr cloud_in, PCXYZ::Ptr cloud_down,
                               double voxel_down_res,
                               double min_x, double max_x,
                               double min_y, double max_y,
                               double min_z, double max_z, bool filter_y)
    {
      PCXYZ::Ptr cloud_filtered(new PCXYZ);
      // filter z
      passThroughFilter(cloud_in,cloud_filtered,std::string("z"),min_z,max_z);
      // filter x
      passThroughFilter(cloud_filtered,cloud_filtered,std::string("x"),min_x,max_x);

      if (filter_y)
      {
        passThroughFilter(cloud_filtered,cloud_filtered,std::string("y"),min_y,max_y);
      }
      downsampleCloud(cloud_filtered, cloud_down, voxel_down_res);
    }

    void passThroughFilter(PCXYZ::Ptr pcl_in, PCXYZ::Ptr pcl_out,
                           std::string fieldName,
                           double fieldMin, double fieldMax,
                           bool keep_organized)
    {
      //helper function for filtering on a PointCloud
      pcl::PassThrough<PtXYZ> pass;
      pass.setInputCloud(pcl_in);
      pass.setKeepOrganized(keep_organized);
      //pass.setUserFilterValue(BACKGROUND_DIST);
      pass.setFilterFieldName(fieldName);
      pass.setFilterLimits(fieldMin, fieldMax);
      //pass.setFilterLimitsNegative (true);
      pass.filter(*pcl_out);
    }

    void passThroughFilter(PCXYZRGB::Ptr pcl_in, PCXYZRGB::Ptr pcl_out,
                           std::string fieldName,
                           double fieldMin, double fieldMax,
                           bool keep_organized)
    {
      //helper function for filtering on a PointCloud
      pcl::PassThrough<PtXYZRGB> pass;
      pass.setInputCloud(pcl_in);
      pass.setKeepOrganized(keep_organized);
      //pass.setUserFilterValue(BACKGROUND_DIST);
      pass.setFilterFieldName(fieldName);
      pass.setFilterLimits(fieldMin, fieldMax);
      //pass.setFilterLimitsNegative (true);
      pass.filter(*pcl_out);
    }

    cv::Mat projectProtoObjectsIntoImage(ProtoObjects& objs, cv::Size img_size, ProjMat projMat)
    {
      cv::Mat obj_img(img_size, CV_8UC1, cv::Scalar(0));
      for(unsigned int i=0; i < objs.size(); ++i)
      {
        projectPointCloudIntoImage(objs[i].cloud, obj_img, projMat, i+1);
      }
      return obj_img;
    }

    cv::Mat projectProtoObjectIntoImage(ProtoObject& obj, cv::Size img_size, ProjMat projMat)
    {
      cv::Mat obj_img(img_size, CV_8UC1, cv::Scalar(0));
      projectPointCloudIntoImage(obj.cloud, obj_img, projMat, 1);
      return obj_img;
    }

    void projectPointCloudIntoImage(PCXYZ::Ptr cloud, cv::Mat& lbl_img, ProjMat projMat, int label)
    {
      Eigen::Vector3d uvd;
      for(int i=0; i < cloud->size(); ++i)
      {
        uvd = projectPointIntoImage(cloud->at(i),projMat);
        lbl_img.at<char>(static_cast<int>(uvd[0]),static_cast<int>(uvd[1])) = label;
      }
    }

    Eigen::Vector3d projectPointIntoImage(PtXYZ pt, ProjMat projMat)
    {
      Eigen::Vector4d pos(pt.x,pt.y,pt.z,1.0);
      Eigen::Vector3d uv = projMat * pos;

      uv(0) = uv(0) / uv(2);
      uv(1) = uv(1) / uv(2);
      return uv;
    }

    void writeCloud(PCXYZRGB::Ptr cloud,
                    std::string file)
    {
      /* cloud - pointcloud to be written to file
         file  - file to save it to
         ---
       saves the cloud */
      //std::string fp = "/home/hunter/catkin_ws/output/data_processing/";
      std::stringstream ss;
      ss << file << ".pcd";
      pcl::io::savePCDFileASCII(ss.str(), *cloud);
    }

    void writeCloud(PCXYZRGB::Ptr cloud,
                    std::string file,
                    int i)
    {
      /* cloud  - pointcloud to be written to file
         file   - file name to save it to
         i      - numbering to add to end of name
         ---
         saves pointcloud with numbering */
      //std::string fp = "/home/hunter/catkin_ws/output/data_processing/";
      std::stringstream ss;
      ss << file << i << ".pcd";
      pcl::io::savePCDFileASCII(ss.str(), *cloud);
    }

    void findNormals(PCXYZRGB::Ptr in_pcl, PCNormal::Ptr normals)
    {
      /* Finds the normal vectors a pcl
         assumes an organized pointcloud */
      pcl::IntegralImageNormalEstimation<PtXYZRGB, PtNormal> ne;
      ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
      ne.setMaxDepthChangeFactor(.02f);
      ne.setNormalSmoothingSize(10.0f);
      ne.setInputCloud(in_pcl);
      ne.compute(*normals);
    }
    /**
     * Compute the bounding box associated with an object cloud using PCA
     *
     * @param obj_color_cloud The cloud to bound
     * @param pose [output] The object's pose
     * @param width [output] The bounding box width (1st principal component range)
     * @param height [output] The bounding box height (2nd principal component range)
     * @param depth [output] The bounding box depth (3rd principal component range)
     */
    void findCloudBoundingBoxPCA(PCXYZRGB::Ptr& obj_color_cloud, geometry_msgs::Pose& pose,
                               double& width, double& height, double& depth,
                               bool visualize_bounding_box=false)
    {
      Eigen::Vector3f eigen_values;
      Eigen::Matrix3f eigen_vectors;
      Eigen::Vector4f centroid;
      // Remove the RGB components from the input cloud
      PCXYZ obj_cloud;
      pcl::copyPointCloud(*obj_color_cloud, obj_cloud);
      pcl::PCA<PtXYZ> pca;
      try{
        pca.setInputCloud(obj_cloud.makeShared());
        //ROS_INFO("Getting mean");
        centroid = pca.getMean();
        //ROS_INFO("Getting eigen values");
        eigen_values = pca.getEigenValues();
        //ROS_INFO("Getting eigen vectors");
        eigen_vectors = pca.getEigenVectors();
      } catch(pcl::InitFailedException ife)
      {
        ROS_INFO("Failed to compute PCA");
        ROS_INFO_STREAM("ife: " << ife.what());
      }
      // Ensure the coordinate system is right handed
      eigen_vectors.col(2) = eigen_vectors.col(0).cross(eigen_vectors.col(1));
      Eigen::Matrix4f proj_transform;
      proj_transform.block<3,3>(0,0) = eigen_vectors.transpose();
      proj_transform.block<3,1>(0,3) = -1.f * (proj_transform.block<3,3>(0,0) * centroid.head<3>());
      // Transform points into eigen space to find extrema and compute ranges
      PCXYZ proj;
      // pca.project(obj_cloud, proj);
      pcl::transformPointCloud(obj_cloud, proj, proj_transform);

      PtXYZ proj_min;
      PtXYZ proj_max;
      pcl::getMinMax3D(proj, proj_min, proj_max);
      width = fabs(proj_max.x-proj_min.x);
      height = fabs(proj_max.y-proj_min.y);
      depth = fabs(proj_max.z-proj_min.z);

      Eigen::Vector3f box_mean_eigenspace = 0.5f*(proj_max.getVector3fMap() +
                                                  proj_min.getVector3fMap());

      // Compute the transform into the object's space
      Eigen::Quaternionf orientation(eigen_vectors);
      orientation.normalize();
      Eigen::Vector3f rectified_centroid = eigen_vectors*box_mean_eigenspace + \
        centroid.head<3>();
      pose.position.x = rectified_centroid[0];
      pose.position.y = rectified_centroid[1];
      pose.position.z = rectified_centroid[2];
      pose.orientation.x = orientation.x();
      pose.orientation.y = orientation.y();
      pose.orientation.z = orientation.z();
      pose.orientation.w = orientation.w();

      if (visualize_bounding_box)
      {
        ROS_INFO_STREAM("eigen values:\n" << eigen_values);
        ROS_INFO_STREAM("eigen vectors\n" << eigen_vectors);
        // ROS_INFO_STREAM("e1 X e2 = " << eig3);
        ROS_INFO_STREAM("pca centroid:\n" << centroid);
        ROS_INFO_STREAM("box_mean:\n" << box_mean_eigenspace);
        ROS_INFO_STREAM("Recovered centroid:\n" << rectified_centroid);
        ROS_INFO_STREAM("Recovered orientation:\n"<< pose.orientation);
        /* visualizes the inputed pointcloud */
        pcl::visualization::PCLVisualizer viewer("PCL Viewer");
        viewer.setBackgroundColor(0.0,0.0,0.7);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(obj_color_cloud);
        viewer.addPointCloud<PtXYZRGB>(obj_color_cloud, rgb, "object_cloud");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
                                                "object_cloud");

        viewer.addCube(rectified_centroid, orientation, width, height, depth);
        while(!viewer.wasStopped())
        {
          viewer.spinOnce();
        }
      }
    }
    }
}
