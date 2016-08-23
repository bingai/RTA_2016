#include <reconstruction/reconstruction.hpp>

#include <pcl/visualization/pcl_visualizer.h>

std::vector<geometry_msgs::Pose> getDemoPoses() {
	std::vector<geometry_msgs::Pose> poses(0);

	// Preparation
	poses.push_back(g1::control::constructPose(
		g1::control::constructPoint(0.692, 0.653, 0.335),
		g1::control::constructQuat(-0.037, 0.695, -0.048, 0.717)
	));

	poses.push_back(g1::control::constructPose(
		g1::control::constructPoint(0.616, 0.043, 0.311),
		g1::control::constructQuat(0.587, 0.706, -0.212, 0.336)
	));

	poses.push_back(g1::control::constructPose(
		g1::control::constructPoint(0.608, -0.125, 0.067),
		g1::control::constructQuat(0.471, 0.764, -0.132, 0.420)
	));

	poses.push_back(g1::control::constructPose(
		g1::control::constructPoint(0.729, -0.202, 0.475),
		g1::control::constructQuat(0.645, 0.734, -0.087, 0.196)
	));

	// Reset perparation
	poses.push_back(g1::control::constructPose(
		g1::control::constructPoint(0.692, 0.653, 0.335),
		g1::control::constructQuat(-0.037, 0.695, -0.048, 0.717)
	));

	return poses;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "reconstruction");
	ros::NodeHandle nh;

	if (argc < 2) {
		ROS_ERROR("Reconstruction name not provided");
		return 1;
	}

	ros::AsyncSpinner spinner(1);
	spinner.start();

	g1::control::Executor::Options opt(nh, "left", "electric", "reflex");


	g1::control::Executor motion_controller(opt);

	motion_controller.resetArms();

	// g1::control::Executor *motion_controller;

	Reconstruction rec(nh, &motion_controller);
	// Reconstruction rec(nh, &motion_controller, "left", "/camera/rgb/image", "/camera/get_camera_intrinsics", "LowResolutionColor", "/camera/depth/points", "/camera_depth_optical_frame", "/camera_color_optical_frame", "/base", "/world_model/update_table_pcd", false);

	rec.run(getDemoPoses(), argv[1]);

	motion_controller.resetArms();

	rec.slam.writeSceneModel("/home/arclab/Desktop/reconstruction_debug_data");

	rec.updateWorldState();

	// rec.slam.writeSceneModel("/home/kzampog/Desktop/recon_data");

	// ros::ServiceClient world_model_client = nh.serviceClient<world_model_msgs::ReconstructionUpdate>("/world_model/update_reconstruction");
	// world_model_msgs::ReconstructionUpdate srv;
	// srv.request.id = argv[2];
	// srv.request.operation = srv.request.DELETE;
	// std::cout << "DELETION" << std::endl;
	// if (world_model_client.call(srv)) {
	// 	ROS_INFO("World model service call succeeded.");
	// 	if (srv.response.success) {
	// 		ROS_INFO("World model update succeeded.");
	// 		return true;
	// 	} else {
	// 		ROS_ERROR("World model update failed.");
	// 	}
	// } else {
	// 	ROS_ERROR("World model service call failed.");
	// }

	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	// viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	// viewer->setBackgroundColor(0, 0, 0);
	// viewer->addCoordinateSystem(0.3);
	// viewer->initCameraParameters();
	// pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb(rec.slam.model.pointCloud);
	// viewer->addPointCloud<pcl::PointXYZRGBNormal>(rec.slam.model.pointCloud, rgb, "cloud");
	// viewer->spin();

	return 0;
}
