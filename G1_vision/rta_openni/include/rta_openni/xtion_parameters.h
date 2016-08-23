//  ================================================================
//  Created by Gregory Kramida on 8/15/16.
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

#include <ros/node_handle.h>

//OpenCV
#include <opencv2/core/core.hpp>

//std
#include <vector>

namespace rta_openni {
	class xtion_parameters {
	public:
		xtion_parameters(const ros::NodeHandle&,
		                 float color_horizontal_field_of_view,
		                 float depth_horizontal_field_of_view);

		cv::Mat depth_camera_matrix;
		cv::Mat depth_distortion_coefficients;
		cv::Mat lo_ir_camera_matrix;
		cv::Mat lo_color_camera_matrix;
		cv::Mat color_distortion_coefficients;
		cv::Mat color_rotation_matrix;
		cv::Mat color_rotation_vector;
		cv::Mat color_translation;
		std::vector<cv::Mat> displacement_maps;
		std::vector<float> displacement_map_distances;
		//coordinates of pixels in depth camera space after perspective transformation (using camera matrix)
		// & undistortion (using distortion parameters)
		cv::Mat depth_ideal_pixel_coordinates;
		bool displacement_maps_loaded;

	private:
		bool read_depth_displacement_maps(const ros::NodeHandle& node_handle);

		bool read_intrinsic_parameters(const ros::NodeHandle& node_handle);

		void generate_xtion_defaults(float color_horizontal_field_of_view,
		                             float depth_horizontal_field_of_view);
	};


}