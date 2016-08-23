	
#ifndef ACTIVE_DETECT_H
#define ACTIVE_DETECT_H

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

#include <tf_conversions/tf_eigen.h>

#include <time.h> 

// #include <scene_reconstruct/SceneReconstruct.h>

#include <Eigen/Geometry> 
#include <attr_detection/AttributeProcess.h>

class ActiveDetect{
  private:
  tf::TransformListener *tf_listener;

  bool capture;
  std::vector<sensor_msgs::PointCloud2ConstPtr> pcdQue;
  int freshNum;

  ros::Subscriber subl;

  bool readTFTransform(const std::string& target_frame, const std::string& source_frame,
    const ros::Time &time, const double & timeout, Eigen::Affine3d & T_Eigen);

  AttributeProcess ap;
  ros::Publisher tf_pub;

  public:
  ActiveDetect(ros::NodeHandle& nh);
  ~ActiveDetect();
  
  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

  // pcl::visualization::CloudViewer viewer2;
  void callback(const sensor_msgs::PointCloud2ConstPtr& cloud);
  void run();
 
  bool addOneView();
  void setCloudCaptureFlag(bool flag);


  void ConvertPCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                             cv::Mat_<cv::Vec3b> &image);
  void ConvertPCLCloud2ColorSeg(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr &in,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out);
 
  ros::ServiceClient update_client_;
  
};


#endif
