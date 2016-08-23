	
#ifndef COLLECT_DATA_H
#define COLLECT_DATA_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/String.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/console/parse.h>

#include <cstdio>
#include <stdio.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <boost/filesystem.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream> 

#include <tf_conversions/tf_eigen.h>


#include <time.h> 
#include <g1_control/executor.h>
#include <g1_control/utilities.h>
#include <Eigen/Geometry> 

class CollectData {
  private:
  tf::TransformListener *tf_listener;

  bool moveDone;
  std::vector<sensor_msgs::PointCloud2ConstPtr> pcdQue;
  int freshNum;

  ros::Subscriber subl;
  Eigen::Quaternionf frontOrientation;
  Eigen::Vector3f handle_centroid;
  Eigen::Vector3f frontNormal;

  ros::Publisher bbox_pub_;

  void addValidView(std::vector<geometry_msgs::Pose>& validviews, 
    geometry_msgs::Pose pose, g1::control::Executor *motion_controller);


  geometry_msgs::Pose generateView(float x_dist, float y_dist, 
    float z_dist, Eigen::Vector3f centroid, Eigen::Vector3f xdir, 
    Eigen::Vector3f ydir, Eigen::Vector3f zdir);


  public:

  CollectData(ros::NodeHandle& nh_);
  ~CollectData();
  
  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

  // pcl::visualization::CloudViewer viewer2;
  void callback(const sensor_msgs::PointCloud2ConstPtr& cloud);
  bool collect();  
  void run();
  void runWithPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud
    , Eigen::Affine3d& T_Eigen);
  void setCloudCaptureFlag(bool flag);

  // void run(g1::control::Executor *motion_controller, 
  //   SceneReconstruct& sr);


  void ConvertPCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                             cv::Mat_<cv::Vec3b> &image);
 
  void startCaptureData(g1::control::Executor *motion_controller);

  ros::ServiceClient update_client_;
  
  std::string filename;
  int showParts;
};


#endif
