#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/CameraInfo.h>
#include "std_msgs/String.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
// #include <baxter_core_msgs/EndpointState.h>
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
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/pcl_search.h>
#include <pcl/features/normal_3d_omp.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>


ros::Publisher tf_pub;
tf::TransformListener *tf_listener; 
bool done = false;
void callback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{

  if ((cloud->width * cloud->height) == 0) {
        return;
   }


  tf_listener->waitForTransform("/base", cloud->header.frame_id, cloud->header.stamp, ros::Duration(3.0));
  tf::StampedTransform transform;
  try {
    tf_listener->lookupTransform ("/base", cloud->header.frame_id, cloud->header.stamp, transform);
  } catch (tf::LookupException &e) {
    ROS_ERROR ("%s", e.what ());
    return ;
  } catch (tf::ExtrapolationException &e) {
    ROS_ERROR ("%s", e.what ());
    return ;
  }


  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2,*cloud_in);
  pcl::PointCloud<pcl::PointXYZRGB> pcl_out;

  Eigen::Affine3d T_Eigen;
  tf::transformTFToEigen (transform, T_Eigen );
  pcl::transformPointCloud(*cloud_in, pcl_out, T_Eigen.cast<float>());


  tf_pub.publish(pcl_out);

  // pcl::PointCloud<pcl::Normal>::Ptr cloud_n (new pcl::PointCloud<pcl::Normal>);
  // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  // pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  // pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> norm_est;
  // norm_est.setSearchMethod(tree);
  // norm_est.setRadiusSearch(0.03);
  // norm_est.setInputCloud(cloud_in);
  // norm_est.compute(*cloud_n);
  // pcl::concatenateFields(*cloud_in, *cloud_n, *cloud_f);

  // pcl::PointCloud<pcl::PointXYZRGBNormal> pcl_out;

  // pcl_ros::transformPointCloudWithNormals("/base", *cloud_f, pcl_out, *tf_listener);
  // if (pcl_out.points.size() > 0 && !done) {
  //   pcl::io::savePCDFile("orig_cloud1.pcd", *cloud_f, true);
  //   pcl::io::savePCDFile("tf_cloud1.pcd", pcl_out, true);
  //   done = true;
  //   std::cout << "done!" << std::endl;
  // }

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  std::cout << "start publish base points" << std::endl;
  ros::NodeHandle nh;
  tf_listener = new tf::TransformListener();
  tf_pub = nh.advertise< pcl::PointCloud<pcl::PointXYZRGB> > ("/camera/base_registered_points", 1);

  ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, callback);
  // ros::Subscriber subCoord = nh.subscribe("/robot/limb/left/endpoint_state", 1, leftEndpointCallback);
  ros::spin();
}