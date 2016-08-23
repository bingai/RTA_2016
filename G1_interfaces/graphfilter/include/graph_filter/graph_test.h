	
#ifndef MAIN_GRAPH_TEST_H
#define MAIN_GRAPH_TEST_H

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


#include <graph_filter/GraphFilter.h>
#include <tf_conversions/tf_eigen.h>


#include <time.h> 

// #include <scene_reconstruct/SceneReconstruct.h>

#include <Eigen/Geometry> 

class GraphTest{
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

  public:
  GraphTest(ros::NodeHandle& nh);
  ~GraphTest();
  
  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

  // pcl::visualization::CloudViewer viewer2;
  void callback(const sensor_msgs::PointCloud2ConstPtr& cloud);
  bool detectGraphTest();  
  void run();
  void runWithPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud
    , Eigen::Affine3d& T_Eigen);
  void setCloudCaptureFlag(bool flag);

  // void run(g1::control::Executor *motion_controller, 
  //   SceneReconstruct& sr);


  std::vector<geometry_msgs::Pose> runViewPlan();

  void ConvertPCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                             cv::Mat_<cv::Vec3b> &image);
  void ConvertPCLCloud2ColorSeg(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr &in,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out);
 
  GraphFilter mcr;
  ros::ServiceClient update_client_;
  
  std::string filename;
  int showParts;
};


#endif
