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

//rta_openni
#include <rta_openni/xtion_parameters.h>
#include <rta_openni/openni_capture.h>
#include <rta_openni/point_cloud_generation.h>

//OpenCV
#include <opencv2/calib3d/calib3d.hpp>


namespace rta_openni {

	xtion_parameters::xtion_parameters(const ros::NodeHandle& node_handle,
	                                   float color_horizontal_field_of_view,
	                                   float depth_horizontal_field_of_view) :
			depth_camera_matrix(),
			depth_distortion_coefficients(),
			lo_ir_camera_matrix(),
			lo_color_camera_matrix(),
			color_distortion_coefficients(),
			color_rotation_matrix(),
			color_rotation_vector(),
			color_translation(),
			displacement_maps(),
			displacement_map_distances(),
			depth_ideal_pixel_coordinates() {
		if (!read_intrinsic_parameters(node_handle)) {
			generate_xtion_defaults(color_horizontal_field_of_view,
			                        depth_horizontal_field_of_view);
		}
		const unsigned int depth_width = openni2_xtion_capture::depth_width;
		const unsigned int depth_height = openni2_xtion_capture::depth_height;
		//for point-cloud undistortion
		depth_ideal_pixel_coordinates = rta_openni::get_ideal_pixel_coordinates(
				cv::Size(depth_width, depth_height),
				depth_camera_matrix,
				depth_distortion_coefficients);

		//================== LOAD DEPTH DISTORTION MAPS ============================================================
		// Load depth distortion maps
		displacement_maps_loaded = read_depth_displacement_maps(node_handle);

	};

	bool xtion_parameters::read_depth_displacement_maps(const ros::NodeHandle& node_handle) {
		std::string depth_displacement_path;
		if (node_handle.getParam("depth_displacement_file_path", depth_displacement_path)) {
			cv::FileStorage fs(depth_displacement_path, cv::FileStorage::READ);
			if (fs.isOpened()) {
				int num_dm_maps;
				fs["num_dm_maps"] >> num_dm_maps;

				displacement_map_distances.resize(static_cast<unsigned long>(num_dm_maps));
				displacement_maps.resize(static_cast<unsigned long>(num_dm_maps));

				for (int ix_dm = 0; ix_dm < num_dm_maps; ix_dm++) {
					fs["dm_" + std::to_string(ix_dm) + "_distance"] >> displacement_map_distances[ix_dm];
					fs["dm_" + std::to_string(ix_dm) + "_multipliers"] >> displacement_maps[ix_dm];
				}
			} else {
				ROS_WARN("Could not open file to load parameters from %s.", depth_displacement_path.c_str());
				return false;
			}

			fs.release();
			return true;
		} else {
			return false;
		}
	}

	bool xtion_parameters::read_intrinsic_parameters(const ros::NodeHandle& node_handle) {
		bool calibration_file_found = false;
		std::string calibration_file_path;
		if (node_handle.getParam("depth_ir_rgb_calibration_file_path", calibration_file_path)) {
			cv::FileStorage fs(calibration_file_path, cv::FileStorage::READ);
			if (fs.isOpened()) {
				calibration_file_found = true;
				cv::FileNode stereo_calib_node = fs["Rig"];
				cv::FileNode cameras_node = stereo_calib_node["Cameras"];
				cv::FileNode depth_camera_node = cameras_node[0];
				cv::FileNode ir_camera_node = cameras_node[1];
				cv::FileNode color_camera_node = cameras_node[2];
				cv::FileNode intrinsics_depth_node = depth_camera_node["Intrinsics"];
				cv::FileNode intrinsics_ir_node = ir_camera_node["Intrinsics"];
				cv::FileNode intrinsics_color_node = color_camera_node["Intrinsics"];
				cv::FileNode extrinsics_node = color_camera_node["Extrinsics"];
				intrinsics_depth_node["intrinsic_mat"] >> depth_camera_matrix;
				intrinsics_depth_node["distortion_coeffs"] >> depth_distortion_coefficients;
				intrinsics_ir_node["intrinsic_mat"] >> lo_ir_camera_matrix;
				intrinsics_color_node["intrinsic_mat"] >> lo_color_camera_matrix;
				intrinsics_color_node["distortion_coeffs"] >> color_distortion_coefficients;
				extrinsics_node["rotation"] >> color_rotation_matrix;
				extrinsics_node["translation"] >> color_translation;
				cv::Rodrigues(color_rotation_matrix, color_rotation_vector);
			} else {
				ROS_INFO("Calibration file not found at '%s'. Using defaults.", calibration_file_path.c_str());
			}
		} else {
			ROS_INFO("The calibration_file_path parameter was not specified. Using defaults.");
		}
		return calibration_file_found;
	}

	inline static cv::Mat build_camera_matrix_from_hfov(unsigned int width, unsigned int height, float hfov) {
		cv::Mat camera_matrix;
		camera_matrix = cv::Mat::zeros(3, 3, CV_64FC1);
		float h_fov = h_fov;
		float calculated_depth_focal_length = width / (2.0f * tan(h_fov / 2.0f));

		// Generate default camera parameters
		float fx = calculated_depth_focal_length; // Horizontal focal length
		float fy = calculated_depth_focal_length; // Vertcal focal length
		float cx = (static_cast<float>(width) - 1.f) / 2.f;  // Center x
		float cy = (static_cast<float>(height) - 1.f) / 2.f; // Center y

		camera_matrix.at<float>(0, 0) = fx;
		camera_matrix.at<float>(1, 1) = fy;
		camera_matrix.at<float>(2, 2) = 1.0;
		camera_matrix.at<float>(0, 2) = cx;
		camera_matrix.at<float>(1, 2) = cy;

		return camera_matrix;
	}

	void xtion_parameters::generate_xtion_defaults(float color_horizontal_field_of_view,
	                                               float depth_horizontal_field_of_view) {
		depth_camera_matrix = build_camera_matrix_from_hfov(openni2_xtion_capture::depth_width,
		                                                       openni2_xtion_capture::depth_height,
		                                                       depth_horizontal_field_of_view);
		depth_distortion_coefficients = cv::Mat::zeros(5, 1, CV_64FC1);
		lo_color_camera_matrix = build_camera_matrix_from_hfov(openni2_xtion_capture::lo_color_width,
		                                                          openni2_xtion_capture::lo_color_height,
		                                                          color_horizontal_field_of_view);
		color_distortion_coefficients = cv::Mat::zeros(5, 1, CV_64FC1);
		color_rotation_matrix = cv::Mat::eye(3, 3, CV_64FC1);
		depth_camera_matrix.copyTo(lo_ir_camera_matrix);
		color_rotation_vector = cv::Mat::zeros(3, 1, CV_64FC1);
		//defaults are some values taken from an Xtion Pro camera.
		color_translation = (cv::Mat_<double>(3, 1) << -0.02660147f, 0.00031961f, -0.00518997f);
	}
}