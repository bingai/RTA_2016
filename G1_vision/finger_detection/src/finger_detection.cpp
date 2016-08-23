#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/pcl_search.h>
#include <pcl/features/normal_3d_omp.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Dense>
#include <pcl/common/intersections.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_clusters.h>


ros::Publisher tf_pub;
tf::TransformListener *tf_listener; 
bool done = false;

Eigen::Affine3d readTFTransform(const std::string& target_frame, const std::string& source_frame,
  const ros::Time &time, const ros::Duration &timeout) {

  Eigen::Affine3d T_Eigen;
  tf_listener->waitForTransform(target_frame, source_frame, time, timeout);
  tf::StampedTransform transform;
  try {
    tf_listener->lookupTransform (target_frame, source_frame, time, transform);
  } catch (tf::LookupException &e) {
    ROS_ERROR ("%s", e.what ());
    return T_Eigen;
  } catch (tf::ExtrapolationException &e) {
    ROS_ERROR ("%s", e.what ());
    return T_Eigen;
  }

  tf::transformTFToEigen (transform, T_Eigen );

  return T_Eigen;
}

void cropPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::Affine3d& T_Eigen) {
  
  Eigen::Matrix4d T = T_Eigen.matrix();
  Eigen::Vector3f center (T(0,3), T(1,3), T(2,3));
  Eigen::Matrix3f rotation = T.block<3,3>(0, 0).cast<float>();
  Eigen::Vector3f x = rotation.col(0);
  Eigen::Vector3f y = rotation.col(1);
  Eigen::Vector3f z = rotation.col(2);

  float yd = center.dot(y) * -1;
  float xd = (center - 0.05*x).dot(x) * -1;
  float zd = (center - 0.05*z).dot(z) * -1;
  
  double yoffset = -1;
  Eigen::Vector3f fingerTip;
  std::vector<int> pointIndice;
  for (int i = 0; i < cloud->points.size(); i++) {
    if (!pcl::isFinite(cloud->points[i]) ) {
      continue;
    }
    // std::cout << "points " << cloud->points[i].getVector3fMap() << std::endl;
    double dist_x = pcl::pointToPlaneDistanceSigned(cloud->points[i],
      x[0], x[1], x[2], xd);
    double dist_y = pcl::pointToPlaneDistanceSigned(cloud->points[i],
      y[0], y[1], y[2], yd);
    double dist_z = pcl::pointToPlaneDistanceSigned(cloud->points[i],
      z[0], z[1], z[2], zd);
    // std::cout << "dist " << dist_x << " " << dist_y << " " << dist_z << std::endl;
    if (dist_x <= 0 || dist_x >= 0.10) {
      continue;
    }
    if (dist_y <= 0 || dist_y >= 0.17) {
      continue;
    }

    if (dist_z <= 0 || dist_z >= 0.10) {
      continue;
    }
    pointIndice.push_back(i);

    if (dist_y > yoffset) {
      yoffset = dist_y;
      fingerTip = cloud->points[i].getVector3fMap();
    }
  }

  // if (pointIndice.size() == 0) {
  //   std::cout << "No points in the hull" << std::endl;
  //   return ;
  // }

  // boost::shared_ptr<std::vector<int> > ind(new std::vector<int>(pointIndice));

  // pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  // ec.setInputCloud(cloud);
  // ec.setIndices(ind);
  // ec.setClusterTolerance(0.01);
  // ec.setMinClusterSize(200);
  // ec.setMaxClusterSize(25000);

  // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
  //   new pcl::search::KdTree<pcl::PointXYZRGB>);
  // tree->setInputCloud(cloud, ind);

  // ec.setSearchMethod(tree);

  // std::vector<pcl::PointIndices> cluster_ind;
  // ec.extract(cluster_ind);

  // std::cout << "cluster number " << cluster_ind.size() << std::endl;

  
  std::cout << "yoffset " << yoffset << std::endl;
  if (yoffset > 0) {
    // std::cout << fingerTip <<std::endl << std::endl;
    static tf::TransformBroadcaster br;
    tf::Transform microTF;
    microTF.setOrigin(tf::Vector3(fingerTip[0], fingerTip[1], fingerTip[2]) );
    tf::Quaternion microQ;
    microQ.setRPY(0, 0, 0);
    microTF.setRotation(microQ);
    // br.sendTransform(tf::StampedTransform(microTF, ros::Time::now(), "/camera_rgb_optical_frame", "/object"));
    br.sendTransform(tf::StampedTransform(microTF, ros::Time::now(), "/base", "/fingerTip"));
  }

}

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{

  if ((cloud->width * cloud->height) == 0) {
        return;
   }



  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud, *cloud_in);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_out(new pcl::PointCloud<pcl::PointXYZRGB>);

  Eigen::Affine3d T_Eigen = readTFTransform("/base", cloud->header.frame_id, 
    cloud->header.stamp, ros::Duration(3.0));
  pcl::transformPointCloud(*cloud_in, *pcl_out, T_Eigen.cast<float>());

  pcl_out->header.frame_id = "/base";
  // tf_pub.publish(*pcl_out);

  Eigen::Affine3d marker = readTFTransform("/base", "ar_marker_2", 
    cloud->header.stamp, ros::Duration(3.0));

  // std::cout << marker.matrix() << std::endl;
  cropPointCloud(pcl_out, marker);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  std::cout << "Try finger detection" << std::endl;
  ros::NodeHandle nh;
  tf_listener = new tf::TransformListener();
  tf_pub = nh.advertise< pcl::PointCloud<pcl::PointXYZRGB> > ("/camera/base_registered_points", 1);

  ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, callback);
  // ros::Subscriber subCoord = nh.subscribe("/robot/limb/left/endpoint_state", 1, leftEndpointCallback);
  ros::spin();
}