//  ================================================================
//  Created by Gregory Kramida on 6/27/16.
//  Copyright (c) 2016 Gregory Kramida
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================
//ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <rta_openni/UseHighResolutionColor.h>
#include <rta_openni/GetCameraIntrinsics.h>
#include <rta_openni/GetCameraExtrinsics.h>
#include <sensor_msgs/point_cloud2_iterator.h>

void color_callback(const sensor_msgs::ImageConstPtr& msg) {
	cv::Mat frame = cv_bridge::toCvShare(msg, "rgb8")->image;
	ROS_INFO("Received color frame: %d x %d", frame.cols, frame.rows);
}

void depth_callback(const sensor_msgs::ImageConstPtr& msg) {
	cv::Mat frame = cv_bridge::toCvShare(msg, "mono16")->image;
	ROS_INFO("Received depth frame: %d x %d", frame.cols, frame.rows);
}

void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
	// read off in XYZ values
	sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
	sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
	sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");
	const float bad_point = std::numeric_limits<float>::quiet_NaN();
	unsigned int total_point_count = cloud_msg->width * cloud_msg->height;
	double ave_x = 0.0;
	double ave_y = 0.0;
	double ave_z = 0.0;
	int ix_pt = 0;
	for (; iter_z != iter_z.end(); ++iter_x, ++iter_y, ++iter_z, ix_pt++) {
		if ((*iter_x) != bad_point) {
			ave_x += *iter_x;
			ave_y += *iter_y;
			ave_z += *iter_z;
			int row = ix_pt / cloud_msg->width;
			int col = ix_pt % cloud_msg->width;
		}
	}
	ave_x /= total_point_count;
	ave_y /= total_point_count;
	ave_z /= total_point_count;

	//ROS_INFO("Received point cloud: %d x %d; x,y,z averages: %f, %f, %f", cloud_msg->width, cloud_msg->height, ave_x, ave_y, ave_z);
	ROS_INFO("Received point cloud: %d x %d", cloud_msg->width, cloud_msg->height);
}

inline
static void intrinsics_call_succeeded(rta_openni::GetCameraIntrinsics& intrinsics_service){
	cv::Mat camera_matrix(3, 3, CV_64FC1);
	cv::Mat distortion_coeffs(static_cast<int>(intrinsics_service.response.distortion_coefficient_array.size()), 1,
	                          CV_64FC1);
	memcpy(camera_matrix.data, intrinsics_service.response.camera_matrix_array.data(),
	       intrinsics_service.response.camera_matrix_array.size() * sizeof(double));
	memcpy(distortion_coeffs.data, intrinsics_service.response.distortion_coefficient_array.data(),
	       intrinsics_service.response.distortion_coefficient_array.size() * sizeof(double));
	std::stringstream ss;
	ss << std::endl << "==Camera matrix:" << std::endl << camera_matrix << std::endl << "==Distortion coefficients:"
			<< distortion_coeffs << std::endl;
	ROS_INFO("Call to camera/get_camera_intrinsics for camera '%s' succeeded.%s",
	         intrinsics_service.request.which_camera.c_str(), ss.str().c_str());
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "listener");
	ros::NodeHandle node_handle;
	image_transport::ImageTransport image_transport(node_handle);
	ros::ServiceClient resolution_client = node_handle.serviceClient<rta_openni::UseHighResolutionColor>(
			"camera/use_high_resolution_color");
	ros::ServiceClient extrinsics_client = node_handle.serviceClient<rta_openni::GetCameraExtrinsics>(
			"camera/get_camera_extrinsics");
	ros::ServiceClient intrinsics_client = node_handle.serviceClient<rta_openni::GetCameraIntrinsics>(
			"camera/get_camera_intrinsics");

	//using service objects directly with the client is a surefire way to call another node's service.
	//There are, however, other ways, e.g. calling by using a specific message class, such as std_msgs::camera_info,
	// & service URI.
	rta_openni::UseHighResolutionColor resolution_service;
	rta_openni::GetCameraExtrinsics extrinsics_service;
	rta_openni::GetCameraIntrinsics intrinsics_service;


	//call to test all services.
	resolution_service.request.use_high_resolution_color = false;
	if (resolution_client.call(resolution_service)) {
		ROS_INFO("Call to camera/use_high_resolution_color succeeded.");
	} else {
		ROS_ERROR("Failed to call resolution_service camera/use_high_resolution_color.");
	}
	if (extrinsics_client.call(extrinsics_service)) {
		cv::Mat color_rotation(3, 3, CV_64FC1);
		cv::Mat color_translation(3, 1, CV_64FC1);
		memcpy(color_rotation.data, extrinsics_service.response.color_rotation.data(),
		       extrinsics_service.response.color_rotation.size() * sizeof(double));
		memcpy(color_translation.data, extrinsics_service.response.color_translation.data(),
		       extrinsics_service.response.color_translation.size() * sizeof(double));
		std::stringstream ss;
		ss << std::endl << "==Rotation matrix: " << std::endl << color_rotation << std::endl << "==Translation vector: " <<
		color_translation << std::endl;
		ROS_INFO("Call to camera/get_camera_extrinsics succeeded.%s", ss.str().c_str());
	} else {
		ROS_ERROR("Failed to call resolution_service camera/get_camera_extrinsics.");
	}
	intrinsics_service.request.which_camera = "LowResolutionColor"; //Can also be "HighResolutionColor", "Depth", "IR"
	if (intrinsics_client.call(intrinsics_service)) {
		intrinsics_call_succeeded(intrinsics_service);
	} else {
		ROS_ERROR("Failed to call resolution_service camera/get_camera_intrinsics.");
	}
	intrinsics_service.request.which_camera = "Depth"; //Can also be "LowResolutionColor", "Depth", "IR"
	if (intrinsics_client.call(intrinsics_service)) {
		intrinsics_call_succeeded(intrinsics_service);
	} else {
		ROS_ERROR("Failed to call resolution_service camera/get_camera_intrinsics.");
	}
	intrinsics_service.request.which_camera = "GobblyGook";
	if (intrinsics_client.call(intrinsics_service)) {
		if (intrinsics_service.response.success == false) {
			ROS_INFO("Call to camera/get_camera_intrinsics failed as expected. Message: '%s'",
			         intrinsics_service.response.message.c_str());
		} else {
			ROS_ERROR("Call to resolution_service camera/get_camera_intrinsics with wrong parameter succeeded.");
		}
	} else {
		ROS_ERROR("Failed to call resolution_service camera/get_camera_intrinsics.");
	}


	image_transport::Subscriber color_subscriber = image_transport.subscribe("camera/rgb/image", 1, color_callback);
	image_transport::Subscriber depth_subscriber = image_transport.subscribe("camera/depth/image", 1, depth_callback);
	ros::Subscriber point_cloud_subscriber = node_handle.subscribe<sensor_msgs::PointCloud2>("camera/depth/points", 1,
	                                                                                         point_cloud_callback);

	ros::Rate loop_rate(5);//hz
	resolution_service.request.use_high_resolution_color = false;
	int counter = 0;
	while (ros::ok()) {
		if (counter % 50 == 0) {
			//switch up the color camera resolution
			resolution_service.request.use_high_resolution_color = !resolution_service.request.use_high_resolution_color;
			if (resolution_client.call(resolution_service)) {
				ROS_INFO("Call to camera/use_high_resolution_color succeeded.");
			} else {
				ROS_ERROR("Failed to call resolution_service camera/use_high_resolution_color.");
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
		counter++;
	}
	return 0;
}