//  ================================================================
//  Created by Gregory Kramida on 7/14/16..
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
#pragma once

//ROS
#include <pcl_ros/point_cloud.h>

//OpenCV
#include <opencv2/core/core.hpp>

typedef sensor_msgs::PointCloud2 PointCloud;

namespace rta_openni {
	cv::Mat get_ideal_pixel_coordinates(const cv::Size& image_size, const cv::Mat& camera_matrix,
	                                    const cv::Mat& distortion_coefficients);

	PointCloud::Ptr generate_xyz_point_cloud(const cv::Mat& depth_frame, const cv::Mat& ideal_pixel_coordinates);

	PointCloud::Ptr generate_xyz_point_cloud(const cv::Mat& depth_frame,
	                                         const cv::Mat& ideal_pixel_coordinates,
	                                         const std::vector<cv::Mat>& dm_maps,
	                                         const std::vector<float>& dm_distances);


	PointCloud::Ptr generate_color_registered_point_cloud(
			const PointCloud::Ptr& xyz_point_cloud, const cv::Mat& color_image, const cv::Mat& intrinsic_matrix,
			const cv::Mat& distortion_coefficients, const cv::Mat& rotation_vector, const cv::Mat& rotation_matrix,
			const cv::Mat& translation);
}