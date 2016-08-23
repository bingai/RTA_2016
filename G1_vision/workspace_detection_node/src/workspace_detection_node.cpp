#include <workspace_detection_node/workspace_detection_node.hpp>
#include <workspace_detection_node/workspace_detection_utilities.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>

void boxToObjectMessage(const g1::vision::Box &box, world_model_msgs::Object &object)
{
	object.primitives.resize(1);
	object.primitive_poses.resize(1);

	// Solid primitive
	shape_msgs::SolidPrimitive sp;
	sp.type = sp.BOX;
	sp.dimensions.resize(3);
	sp.dimensions[sp.BOX_X] = box.size_(0);
	sp.dimensions[sp.BOX_Y] = box.size_(1);
	sp.dimensions[sp.BOX_Z] = box.size_(2);
	object.primitives[0] = sp;

	// Pose
	tf::poseEigenToMsg(box.pose_.cast<double>(), object.primitive_poses[0]);

	// Id
	object.id = box.id_;
}

WorkspaceDetectionNode::WorkspaceDetectionNode() :
	nh(),
	wd_server(nh.advertiseService("/workspace_detection_service", &WorkspaceDetectionNode::replyToRequest, this)),
	pcd_client(nh.serviceClient<world_model_msgs::ReconstructedPointCloudQuery>("/world_model/get_reconstructed_point_cloud")),
	object_update_client(nh.serviceClient<world_model_msgs::UpdateStatesObjects>("/world_model/update_states_objects"))
{
	ROS_INFO("Waiting for world model services:");
	ROS_INFO("  %s", pcd_client.getService().c_str());
	ROS_INFO("  %s", object_update_client.getService().c_str());
	pcd_client.waitForExistence();
	object_update_client.waitForExistence();
	ROS_INFO("Ready to process.");
}

bool WorkspaceDetectionNode::replyToRequest(workspace_detection_node::WorkspaceDetectionRequest &req, workspace_detection_node::WorkspaceDetectionResponse &res)
{
	ROS_INFO("Received request of type %d for workspace \"%s\" with point cloud \"%s\".", req.space_type, req.space_id.c_str(), req.point_cloud_id.c_str());

	space_type = req.space_type;
	space_id = req.space_id;

	// Get cloud from world model
	world_model_msgs::ReconstructedPointCloudQuery pcd_srv;
	pcd_srv.request.id = req.point_cloud_id;
	if (!pcd_client.call(pcd_srv)) {
		ROS_ERROR("World model service call failed (/world_model/get_reconstructed_point_cloud).");
		res.success = false;
		return true;
	}
	if (!pcd_srv.response.success) {
		ROS_ERROR("Failed to retrieve point cloud for \"%s\" from the world model.", req.point_cloud_id.c_str());
		res.success = false;
		return true;
	}

	reconstructed_cloud.reset(new pcl::PointCloud<PointNC>);
	pcl::fromROSMsg(pcd_srv.response.point_cloud, *reconstructed_cloud);

	ROS_INFO("Processing...");
	if (!process()) {
		ROS_ERROR("Processing failed!");
		res.success = false;
		return true;
	}
	ROS_INFO("Processing succeeded.");

	// Push results to world model
	world_model_msgs::UpdateStatesObjects obj_srv;
	obj_srv.request.operation = obj_srv.request.UPDATE;
	obj_srv.request.objects_info = object_msgs;
	if (!object_update_client.call(obj_srv)) {
		ROS_ERROR("World model service call failed (/world_model/update_states_objects).");
		res.success = false;
		return true;
	}
	if (!obj_srv.response.success) {
		ROS_ERROR("World model failed to update \"%s\" space.", req.space_id.c_str());
		res.success = false;
		return true;
	}	

	ROS_INFO("World model updated successfully.");
	res.success = true;
	return true;
}

bool WorkspaceDetectionNode::process()
{
	bool success;
	if (space_type == workspace_detection_node::WorkspaceDetectionRequest::TABLE_TOP) {
		g1::vision::Box table_box;
		success = g1::vision::findTableBox(reconstructed_cloud, 1.0, space_id, table_box);
		if (success) {
			object_msgs.resize(1);
			boxToObjectMessage(table_box, object_msgs[0]);
		}
	} else if (space_type == workspace_detection_node::WorkspaceDetectionRequest::FRIDGE_INTERIOR) {
		g1::vision::Box fridge_in_box;
		success = g1::vision::findFridgeBox(reconstructed_cloud, true, space_id, fridge_in_box);
		if (success) {
			object_msgs.resize(1);
			boxToObjectMessage(fridge_in_box, object_msgs[0]);
		}
	} else if (space_type == workspace_detection_node::WorkspaceDetectionRequest::FRIDGE_EXTERIOR) {
		g1::vision::Box fridge_out_box;
		success = g1::vision::findFridgeBox(reconstructed_cloud, false, space_id, fridge_out_box);
		if (success) {
			object_msgs.resize(1);
			boxToObjectMessage(fridge_out_box, object_msgs[0]);
		}
	} else {
		ROS_ERROR("Unknown space type.");
		return false;
	}

	return success;
}

void WorkspaceDetectionNode::run()
{
	ros::Rate loop_rate(5);
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init (argc, argv, "workspace_detection_node");

	WorkspaceDetectionNode wd;
	wd.run();

	return 0;
}
