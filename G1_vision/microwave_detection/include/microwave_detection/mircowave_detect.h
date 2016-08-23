	
#ifndef MAIN_MICROWAVE_DETECT_H
#define MAIN_MICROWAVE_DETECT_H

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
#include <pcl/filters/filter.h>

#include <cstdio>
#include <stdio.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <boost/filesystem.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <g1_control/executor.h>
#include <g1_control/utilities.h>

#include <microwave_detection/microwave_reconstruct.h>
#include <tf_conversions/tf_eigen.h>

// #include <scene_reconstruct/SceneReconstruct.h>

#include <Eigen/Geometry> 
#include <world_model_msgs/UpdateStatesObjects.h>

class Microwave{
  private:
  tf::TransformListener *tf_listener;

  bool moveDone;
  std::vector<sensor_msgs::PointCloud2ConstPtr> pcdQue;
  int freshNum;

  ros::Subscriber subl;
  Eigen::Quaternionf frontOrientation;
  Eigen::Vector3f handle_centroid;
  Eigen::Vector3f frontNormal;

  geometry_msgs::Pose generateView(float x, float y, float z);
  void addValidView(std::vector<geometry_msgs::Pose>& validviews, 
    geometry_msgs::Pose pose, g1::control::Executor *motion_controller);

  public:
  Microwave(ros::NodeHandle& nh);
  ~Microwave();
  

  // pcl::visualization::CloudViewer viewer2;
  void callback(const sensor_msgs::PointCloud2ConstPtr& cloud);
  bool detectMicrowave();  
  void run(g1::control::Executor *motion_controller);
  void runWithPointCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr &cloud);
  
  // void run(g1::control::Executor *motion_controller, 
  //   SceneReconstruct& sr);


  std::vector<geometry_msgs::Pose> runViewPlan(g1::control::Executor *motion_controller);

  void ConvertPCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                             cv::Mat_<cv::Vec3b> &image);
  void ConvertPCLCloud2ColorSeg(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr &in,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out);
  bool updateWorldStates();
  MicrowaveRect mcr;
  ros::ServiceClient update_client_;


};


#endif
