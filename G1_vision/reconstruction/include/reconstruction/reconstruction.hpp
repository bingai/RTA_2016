#ifndef RECONSTRUCTION_HPP
#define RECONSTRUCTION_HPP

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>

#include <sisyphus/sift_rgbd_slam.hpp>
#include <g1_control/executor.h>

bool getCameraExtrinsics(ros::ServiceClient &extrinsics_client, cv::Mat &R, cv::Mat &t);

bool getCameraIntrinsics(ros::ServiceClient &intrinsics_client, const std::string &cam_id, cv::Mat &K, cv::Mat &d);

class Reconstruction {
private:
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> imagePointCloudSyncPolicy;

	ros::NodeHandle node_handle;
	tf::TransformListener *tf_listener;
	message_filters::Subscriber<sensor_msgs::Image> *image_sub;
	message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub;
	message_filters::Synchronizer<imagePointCloudSyncPolicy> *image_cloud_sync;
	g1::control::Executor *motion_controller;
	ros::ServiceClient world_model_client;

	std::string hand_id;
	std::string reference_frame;
	std::string cloud_source_frame;
	std::string cloud_target_frame;
	std::string reconstruction_name;

	Eigen::Matrix4f cloud_transform;
	cv::Mat intrinsics_matrix;
	cv::Mat distortion_coefficients;

	bool manual_spinning;
	boost::mutex data_access;
	bool new_data;
	ros::Time data_timestamp;
	cv::Mat image_curr;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_curr;

	void waitForNewData();
	static void imagePointCloudCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg, Reconstruction *rec);

public:
	SLAM slam;

	Reconstruction();
	Reconstruction	(	ros::NodeHandle &nh,
						g1::control::Executor *motion_control,
						const std::string &hand_name = "left",
						const std::string &img_topic = "/camera/rgb/image",
						const std::string &intr_service = "/camera/get_camera_intrinsics",
						const std::string &intr_cam_id = "LowResolutionColor",
						const std::string &pcd_topic = "/camera/depth/points",
						const std::string &pcd_src_frame = "/camera_depth_optical_frame",
						const std::string &pcd_dst_frame = "/camera_color_optical_frame",
						const std::string &ref_frame = "/base",
						const std::string &wm_service = "/world_model/update_reconstruction",
						bool man_spin = false
					);
	~Reconstruction();

	void captureView();

	bool run(const std::vector<geometry_msgs::Pose> &target_poses, const std::string &rec_name = "reconstruction");
	bool updateWorldState();
};

#endif /* RECONSTRUCTION_HPP */
