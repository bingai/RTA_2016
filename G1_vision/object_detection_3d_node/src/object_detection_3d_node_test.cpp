// Project includes
#include <object_detection_3d_node/ObjectDetection3dService.h>

// ROS
#include <pcl_conversions/pcl_conversions.h>    // This is required for PCL to ROS conversions
#include <ros/ros.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

// Namaris
#include <namaris/utilities/pcl_typedefs.hpp>

int main(int argc, char **argv)
{
  /*
  std::cout << "Don't forget to launch a fake transform publsiher!" << std::endl;
  std::cout << "  rosrun tf static_transform_publisher 0 0 0 0 0 0 base lol 100" << std::endl;
  
  //----------------------------------------------------------------------------
  // Read data
  //----------------------------------------------------------------------------
  
  if (argc < 2)
  {
    std::cout << "You must provide a pointcloud filename" << std::endl;
    return -1;
  }
    
  std::cout << "Loading data..." << std::endl;  
  
  // Read cloud
  std::string cloudFilename = argv[1];
  pcl::PointCloud<PointNC>::Ptr reconstructedCloud (new pcl::PointCloud<PointNC>);
  if (pcl::io::loadPCDFile(cloudFilename, *reconstructedCloud))
  {
    std::cout << "Could not load pointcloud from file:" << std::endl;
    std::cout << cloudFilename << std::endl;
    return -1;
  }
  
  std::cout << "  Done!" << std::endl;  
  */

  //----------------------------------------------------------------------------
  // Initialize node and client
  //----------------------------------------------------------------------------
  
  ros::init(argc, argv, "table_segmentation_node_test");
  ros::NodeHandle n;

  // Add sample node client
  ros::ServiceClient client = n.serviceClient<object_detection_3d_node::ObjectDetection3dService>("object_detection_3d_service");
  object_detection_3d_node::ObjectDetection3dService srv;
  
  // Wait for all necessary servers to establish communications
  client.waitForExistence();
  
  //----------------------------------------------------------------------------
  // Send request
  //----------------------------------------------------------------------------

  // Construct message
  // pcl::toROSMsg (*reconstructedCloud, srv.request.reconstruction.model_point_cloud);
  // srv.request.reconstruction.model_point_cloud.header.frame_id = "/base";

  srv.request.reconstruction_id = argv[1];
  
  // Send request
  if (!client.call (srv))
  {
    ROS_ERROR("Failed to get response. Did the server node die?");
  }
  else
  {
    ROS_INFO("Request sent, response received");
    ROS_INFO("Found %d objects", static_cast<int>(srv.response.objects.size()));
  }
  return 0;
}