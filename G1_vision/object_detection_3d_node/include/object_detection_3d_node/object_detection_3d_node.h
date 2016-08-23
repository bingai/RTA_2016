#ifndef OBJECT_DETECTION_3D_NODE_H
#define OBJECT_DETECTION_3D_NODE_H

// ROS includes
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PolygonStamped.h>

// RVIZ visual tools
#include <rviz_visual_tools/rviz_visual_tools.h>

// Object detection 3d include
#include <object_detection_3d/object_detection_3d.h>

// Service includes
#include <world_model_msgs/ReconstructionQuery.h>
#include <object_cls_msgs/GetObjectClass.h>
#include <world_model_msgs/UpdateStatesObjects.h>
#include <object_detection_3d_node/ObjectDetection3dService.h>

// PCl includes
#include <pcl/point_cloud.h>

// Namaris includes
#include <namaris/utilities/pcl_typedefs.hpp>

class ObjectDetection3dNode
{
public:
  
  // Constructor
  ObjectDetection3dNode ();
  
  // Destructor
  ~ObjectDetection3dNode () {};
    
  // Process service request
  bool replyToRequest ( object_detection_3d_node::ObjectDetection3dServiceRequest  &req,
                        object_detection_3d_node::ObjectDetection3dServiceResponse &res
                      );
  
  // Do the work!
  bool process ();  
  
  // Run
  void run();

private:
    
  // ROS handle
  ros::NodeHandle n_;
  
  // Service for detecting objects
  ros::ServiceServer service_;

  // Client for getting reconstruction results from world model
  ros::ServiceClient get_reconstruction_client_;
  
  // Client for getting 2d recognition result
  ros::ServiceClient object_recognition_2d_client_;

  // Client for publishing detection results to world model
  ros::ServiceClient update_objects_client_;
  
  // For visualizing things in rviz
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;  
    
  // Segmented tabletop publisher
  ros::Publisher tabletop_cloud_publisher_;

  // Reconstruction message from the world model
  reconstruction_msgs::Reconstruction reconstruction_msg_;
  
  // Segmented tabletop ROS message
  sensor_msgs::PointCloud2 tabletop_cloud_msg_;
  
  // Segmented tabletop publisher
  ros::Publisher tabletop_polygon_publisher_;
  
  // Table plane message
  geometry_msgs::PolygonStamped tabletop_polygon_msg_;
  
  // Message containing detected objects
  std::vector<world_model_msgs::Object> object_msg_;
  
  // Input pointcloud
  pcl::PointCloud<PointNC>::Ptr reconstructed_cloud_;
  
  // Output pointcloud
  pcl::PointCloud<PointNC>::Ptr tabletop_cloud_;
  
  // Table plane coefficients
  Eigen::Vector4f table_plane_coefficients_;

  // Output pointcloud
  pcl::PointCloud<PointNC>::Ptr tabletop_polygon_;
  
  // Object cylinders
  std::vector<g1::vision::Cylinder> object_cylinders_;

  // Object boxes
  std::vector<g1::vision::Box> object_boxes_;

  // Reconstruction image view bounding boxes
  std::vector<std::vector<std::vector<int> > > bounding_boxes_2d_;

  // Object messages
  std::vector<world_model_msgs::Object> objects_msg_;
};

#endif  // OBJECT_DETECTION_3D_NODE_H
