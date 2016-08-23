#include <ros/ros.h>

#include <sisyphus/sift_rgbd_slam.hpp>
#include <reconstruction/reconstruction_utilities.hpp>

#include <world_model_msgs/ReconstructionUpdate.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "reconstruction_dummy");
	ros::NodeHandle nh;

	if (argc < 3) {
		ROS_ERROR("Missing command line parameters.");
		return 1;
	}

	ros::ServiceClient world_model_client = nh.serviceClient<world_model_msgs::ReconstructionUpdate>("/world_model/update_reconstruction");

	SLAM slam;
	slam.readSceneModel(argv[1]);
	slam.model.pointCloud->header.frame_id = "/base";

	world_model_msgs::ReconstructionUpdate srv;

	srv.request.id = argv[2];
	srv.request.operation = srv.request.UPDATE;
	SLAMObjectToReconstructionMessage(slam, srv.request.reconstruction);

	if (world_model_client.call(srv)) {
		ROS_INFO("World model service call succeeded.");
		if (srv.response.success) {
			ROS_INFO("World model update succeeded.");
			return true;
		} else {
			ROS_ERROR("World model update failed.");
		}
	} else {
		ROS_ERROR("World model service call failed.");
	}

	return 0;
}
