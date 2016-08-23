#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf_conversions/tf_eigen.h>

#include <rta_openni/GetCameraIntrinsics.h>
#include <rta_openni/GetCameraExtrinsics.h>
#include <world_model_msgs/ReconstructionUpdate.h>

#include <sisyphus/pointcloud_utilities.hpp>

#include <reconstruction/reconstruction.hpp>
#include <reconstruction/reconstruction_utilities.hpp>

bool getCameraExtrinsics(ros::ServiceClient &extrinsics_client, cv::Mat &R, cv::Mat &t) {
	rta_openni::GetCameraExtrinsics extrinsics_service;
	if (extrinsics_client.call(extrinsics_service)) {
		R = cv::Mat(3, 3, CV_64FC1);
		t = cv::Mat(3, 1, CV_64FC1);
		memcpy(R.data, extrinsics_service.response.color_rotation.data(),
		       extrinsics_service.response.color_rotation.size() * sizeof(double));
		memcpy(t.data, extrinsics_service.response.color_translation.data(),
		       extrinsics_service.response.color_translation.size() * sizeof(double));
		R.convertTo(R, CV_32F);
		t.convertTo(t, CV_32F);

		ROS_INFO("Call to extrinsics service succeeded.");
		return true;
	} else {
		ROS_ERROR("Failed to call extrinsics service.");
		return false;
	}
}

bool getCameraIntrinsics(ros::ServiceClient &intrinsics_client, const std::string &cam_id, cv::Mat &K, cv::Mat &d) {
	rta_openni::GetCameraIntrinsics intrinsics_service;
	intrinsics_service.request.which_camera = cam_id;
	if (intrinsics_client.call(intrinsics_service)) {
		K = cv::Mat(3, 3, CV_64FC1);
		d = cv::Mat(static_cast<int>(intrinsics_service.response.distortion_coefficient_array.size()), 1, CV_64FC1);
		memcpy(K.data, intrinsics_service.response.camera_matrix_array.data(), intrinsics_service.response.camera_matrix_array.size() * sizeof(double));
		memcpy(d.data, intrinsics_service.response.distortion_coefficient_array.data(), intrinsics_service.response.distortion_coefficient_array.size() * sizeof(double));
		K.convertTo(K, CV_32F);
		d.convertTo(d, CV_32F);

		ROS_INFO("Call to intrinsics service succeeded.");
		return true;
	} else {
		ROS_ERROR("Failed to call intrinsics service.");
		return false;
	}
}

Reconstruction::Reconstruction() {
	tf_listener = NULL;
	image_sub = NULL;
	cloud_sub = NULL;
	image_cloud_sync = NULL;
	ROS_ERROR("Too few constructor arguments for Reconstruction instance.");
}

Reconstruction::Reconstruction	(	ros::NodeHandle &nh,
									g1::control::Executor *motion_control,
									const std::string &hand_name,
									const std::string &img_topic,
									const std::string &intr_service,
									const std::string &intr_cam_id,
									const std::string &pcd_topic,
									const std::string &pcd_src_frame,
									const std::string &pcd_dst_frame,
									const std::string &ref_frame,
									const std::string &wm_service,
									bool man_spin
								) {

	node_handle = nh;
	motion_controller = motion_control;
	hand_id = hand_name;
	reference_frame = ref_frame;
	cloud_source_frame = pcd_src_frame;
	cloud_target_frame = pcd_dst_frame;
	// reconstruction_name = "reconstruction";

	manual_spinning = man_spin;

	tf_listener = new tf::TransformListener();

	image_sub = new message_filters::Subscriber<sensor_msgs::Image>(node_handle, img_topic, 1);
	cloud_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle, pcd_topic, 1);
	image_cloud_sync = new message_filters::Synchronizer<imagePointCloudSyncPolicy>(imagePointCloudSyncPolicy(10), *image_sub, *cloud_sub);
	image_cloud_sync->registerCallback(boost::bind(&imagePointCloudCallback, _1, _2, this));

	ros::ServiceClient intrinsics_client = node_handle.serviceClient<rta_openni::GetCameraIntrinsics>(intr_service);

	getCameraIntrinsics(intrinsics_client, intr_cam_id, intrinsics_matrix, distortion_coefficients);

	world_model_client = node_handle.serviceClient<world_model_msgs::ReconstructionUpdate>(wm_service);

	tf::StampedTransform extr;
	ros::Time t = ros::Time(0);
	if (!tf_listener->waitForTransform(cloud_target_frame, cloud_source_frame, t, ros::Duration(7.0))) {
		ROS_ERROR("Unable to get point cloud transformation to target frame from tf.");
	}
	tf_listener->lookupTransform(cloud_target_frame, cloud_source_frame, t, extr);

	Eigen::Affine3d aff;
	tf::transformTFToEigen(extr, aff);
	cloud_transform = aff.matrix().cast<float>();

	new_data = false;
}

Reconstruction::~Reconstruction() {
	delete tf_listener;
	delete image_cloud_sync;
	delete cloud_sub;
	delete image_sub;
}

void Reconstruction::imagePointCloudCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg, Reconstruction *rec){
	rec->data_access.lock();
	cv::Mat img = cv_bridge::toCvCopy(rgb_msg)->image;
	rec->image_curr = img;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*cloud_msg, *cloud);
	rec->cloud_curr = cloud;
	rec->data_timestamp = cloud_msg->header.stamp;
	rec->new_data = true;
	rec->data_access.unlock();
}

void Reconstruction::waitForNewData() {
	data_access.lock();
	new_data = false;
	data_access.unlock();
	while (!new_data) {
		ros::Duration(0.1).sleep();
		if (manual_spinning) {
			ros::spinOnce();
		}
	}
}

void Reconstruction::captureView() {
	waitForNewData();
	data_access.lock();

	pcl::transformPointCloud(*cloud_curr, *cloud_curr, cloud_transform);
	cloud_curr->header.frame_id = cloud_target_frame;
	cloud_curr = coordinateRangeClipPointCloud<pcl::PointXYZ>(cloud_curr, true, -2, 2, -2, 2, 0.001, 1.8);

	tf::StampedTransform tf_pose;
	if (!tf_listener->waitForTransform(reference_frame, cloud_target_frame, data_timestamp, ros::Duration(2.0))) {
		ROS_ERROR("Unable to get camera pose from tf.");
	}
	tf_listener->lookupTransform(reference_frame, cloud_target_frame, data_timestamp, tf_pose);

	Eigen::Affine3d aff;
	tf::transformTFToEigen(tf_pose, aff);
	Eigen::Matrix4f pose = aff.matrix().cast<float>();

	// DEBUG WITHOUT ROBOT
	// pose = -Eigen::Matrix4f::Identity();
	// pose(3,3) = 1;

	slam.enqueueView(slam.createView(cloud_curr, image_curr, intrinsics_matrix, distortion_coefficients, pose));
	new_data = false;

	data_access.unlock();

	ROS_INFO("Captured view for reconstruction.");
}

bool Reconstruction::run(const std::vector<geometry_msgs::Pose> &target_poses, const std::string &rec_name) {

	reconstruction_name = rec_name;

	motion_controller->reset();
	boost::shared_ptr<g1::control::Move> move_ptr;
	std::string pref = "view ";
	for (int i = 0; i < target_poses.size(); ++i) {
		move_ptr.reset(new g1::control::Move(pref + std::to_string(i)));
		move_ptr->setTargetPoses(hand_id, std::vector<geometry_msgs::Pose>(1,target_poses[i]));
		motion_controller->addActionTarget(move_ptr);
		if (motion_controller->run()) {
			ROS_INFO("Moved to pose %d.", i);
		} else {
			ROS_ERROR("Execution failed.");
		}
		ros::Duration(2.0).sleep();
		if ((i > 0) && (i < target_poses.size()-1)) {
			captureView();
		}
	}

	int n_success = slam.integrateAllQueuedViews();
	slam.model.pointCloud->header.frame_id = reference_frame;

	ROS_INFO("Integrated %d of %d captured views.", n_success, static_cast<int>(target_poses.size() - 2));
	return n_success == (target_poses.size() - 2);
}

bool Reconstruction::updateWorldState() {
	world_model_msgs::ReconstructionUpdate srv;

	srv.request.id = reconstruction_name;
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
	return false;
}
