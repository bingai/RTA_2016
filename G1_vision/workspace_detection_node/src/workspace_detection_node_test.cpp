#include <ros/ros.h>
#include <workspace_detection_node/WorkspaceDetection.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "workspace_detection_node_test");
	ros::NodeHandle nh;

	ros::ServiceClient wd_client = nh.serviceClient<workspace_detection_node::WorkspaceDetection>("/workspace_detection_service");

	workspace_detection_node::WorkspaceDetection srv;
	srv.request.point_cloud_id = argv[1];
	srv.request.space_id = argv[2];
	srv.request.space_type = std::stoi(argv[3]);

	if (!wd_client.call(srv)) {
		ROS_ERROR("Failed to get response from workspace_detection_node.");
		return 0;
	}
	if (!srv.response.success) {
		ROS_ERROR("Service call to workspace_detection_node failed to update the world model.");
		return 0;
	} else {
		ROS_INFO("World model updated by workspace_detection_node.");
	}

	return 0;
}
