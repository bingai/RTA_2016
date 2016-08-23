#ifndef WORKSPACE_DETECTION_NODE_HPP
#define WORKSPACE_DETECTION_NODE_HPP

#include <ros/ros.h>

#include <world_model_msgs/ReconstructedPointCloudQuery.h>
#include <world_model_msgs/UpdateStatesObjects.h>
#include <workspace_detection_node/WorkspaceDetection.h>

#include <pcl/point_cloud.h>
#include <namaris/utilities/pcl_typedefs.hpp>

class WorkspaceDetectionNode {
public:
	WorkspaceDetectionNode();
	void run();
	bool process();
	bool replyToRequest(workspace_detection_node::WorkspaceDetectionRequest &req, workspace_detection_node::WorkspaceDetectionResponse &res);

private:
	ros::NodeHandle nh;

	ros::ServiceServer wd_server;
	ros::ServiceClient pcd_client;
	ros::ServiceClient object_update_client;

	int space_type;
	std::string space_id;
	pcl::PointCloud<PointNC>::Ptr reconstructed_cloud;
	std::vector<world_model_msgs::Object> object_msgs;
};

#endif /* WORKSPACE_DETECTION_NODE_HPP */
