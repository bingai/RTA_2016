//  ================================================================
// Created by Gregory Kramida on 7/14/16.
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

//local
#include <rta_openni/point_cloud_generation.h>

//ROS
#include <sensor_msgs/point_cloud2_iterator.h>

//OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>


namespace rta_openni {

	//--------------------------------------------------------------------------
	// Undistortion
	//--------------------------------------------------------------------------

	/** \brief Get the ideal coordinates for all pixels in an image.
	 *  \param[in]  image_size size of an image
	 *  \param[in]  K camera matrix
	 *  \param[in]  d distortion coefficients
	 *  \return ideal_pixel_coordinates ideal coordinates of pixels stored in a 1xN CV_32FC2 matrix, column-major order.
	 *          First coordinate corresponds to x and second to y.
	 */
	cv::Mat get_ideal_pixel_coordinates(const cv::Size& image_size, const cv::Mat& camera_matrix,
	                                    const cv::Mat& distortion_coefficients) {
		cv::Mat pixel_coordinates(image_size.area(), 1, CV_32FC2);

		cv::MatIterator_<cv::Vec2f> xy_it = pixel_coordinates.begin<cv::Vec2f>();
		for (int y = 0; y < image_size.height; y++) {
			for (int x = 0; x < image_size.width; x++, xy_it++) {
				(*xy_it)[0] = static_cast<float>(x);
				(*xy_it)[1] = static_cast<float>(y);
			}
		}

		cv::Mat ideal_pixel_coordinates;
		cv::undistortPoints(pixel_coordinates, ideal_pixel_coordinates, camera_matrix, distortion_coefficients);
		return ideal_pixel_coordinates;
	}

	/** \brief Given a value find positions of nearest smaller and nearest greater
     * value in a vector. If only
     *  \param[in] vec    vector
     *  \param[in] value  query value
     *  \return indices of smaller and greater values. If one of them is not available -1 is returned.
     *  \note udenfined behavior if vector contains NaNs.
     */
	template<typename Scalar>
	inline
	std::pair<int, int> nearest_values(const std::vector<Scalar>& vec, const Scalar value) {
		float minDistanceSmaller = std::numeric_limits<Scalar>::max();
		float minDistanceGreater = std::numeric_limits<Scalar>::max();
		int nearestSmallerIndex = -1, nearestGreaterIndex = -1;

		for (size_t i = 0; i < vec.size(); i++) {
			if (vec[i] < value) {
				Scalar distance = value - vec[i];
				if (distance < minDistanceSmaller) {
					minDistanceSmaller = distance;
					nearestSmallerIndex = i;
				}
			} else if (vec[i] > value) {
				Scalar distance = vec[i] - value;
				if (distance < minDistanceGreater) {
					minDistanceGreater = distance;
					nearestGreaterIndex = i;
				}
			} else if (vec[i] == value) {
				return std::pair<int, int>(i, i);
			}
		}
		return std::pair<int, int>(nearestSmallerIndex, nearestGreaterIndex);
	}

	inline
	float get_depth_modifier(float source_depth, int x, int y,
	                         const std::vector<cv::Mat>& dm_maps,
	                         const std::vector<float>& dm_distances) {
		// Update map
		float d_modifier = 1.0f;
		std::pair<int, int> map_indices = nearest_values(dm_distances, source_depth);

		if (map_indices.first != -1 && map_indices.second != -1) {
			if (map_indices.first == map_indices.second) {
				d_modifier = dm_maps[map_indices.first].at<float>(y, x);
			} else {
				float weight = (source_depth - dm_distances[map_indices.first]) /
				               (dm_distances[map_indices.second] - dm_distances[map_indices.first]);
				d_modifier = dm_maps[map_indices.first].at<float>(y, x) * (1 - weight) +
				             dm_maps[map_indices.second].at<float>(y, x) * weight;
			}
		} else if (map_indices.first != -1 && map_indices.second == -1) {
			d_modifier = dm_maps[map_indices.first].at<float>(y, x);
		} else if (map_indices.first == -1 && map_indices.second != -1) {
			d_modifier = dm_maps[map_indices.second].at<float>(y, x);
		}

		return d_modifier;

	}


	/** \brief Correct depth distortion in a kinect depth image.
     *  \param[in]  depth_frame OpenCV depth image (CV_16U where depth is expressed in milimetres)
     *  \param[out] depth_undistorted a 1xN CV_32FC2 matrix where of ideal coordinates of depth image pixels. Pixels are ordered by scanning the rows.
     *  \param[out] cloud PCL pointcloud
     */
	inline
	cv::Mat correct_depth_distortion(const cv::Mat& depth_frame, cv::Mat& depth_corrected,
	                                 const std::vector<cv::Mat>& dm_maps,
	                                 const std::vector<float>& dm_distances) {
		cv::Mat dm_used = cv::Mat::ones(depth_frame.size(), CV_32F);

		// Check input
		if (depth_frame.type() != CV_16U) {
			std::cout << "[correct_depth_distortion] depth image must be of type CV_16U." << std::endl;
			std::abort();
		}

		if (dm_maps.size() == 0) {
			std::cout
					<< "[correct_depth_distortion] depth multiplier maps are empty returning depth unmodified."
					<< std::endl;
			depth_corrected = depth_frame.clone();
			return dm_used;
		}

		if (dm_maps.size() != dm_distances.size()) {
			std::cout
					<< "[correct_depth_distortion] number of depth multiplier maps and distances must be the same."
					<< std::endl;
			std::abort();
		}

		if (dm_maps[0].size() != depth_frame.size()) {
			std::cout
					<< "[correct_depth_distortion] size of depth map and depth multiplier maps must be the same."
					<< std::endl;
			std::abort();
		}

		// Correct depth
		depth_corrected = cv::Mat(depth_frame.size(), depth_frame.type());

		for (int y = 0; y < depth_frame.rows; y++) {
			const uint16_t* depth_row = depth_frame.ptr<uint16_t>(y);
			unsigned short* depth_corrected_row = depth_corrected.ptr<uint16_t>(y);
			for (int x = 0; x < depth_frame.cols; x++) {
				float depth = static_cast<float>(depth_row[x]) / 1000.0f;
				float depth_modifier = 1.0f;
				if (depth != 0.0) {
					depth_modifier = get_depth_modifier(depth, x, y, dm_maps, dm_distances);
				}
				depth_corrected_row[x] = static_cast<uint16_t>(depth * depth_modifier * 1000.0f);
				dm_used.at<float>(y, x) = depth_modifier;
			}
		}
		return dm_used;
	}

	static
	inline void initialize_xyz_point_cloud(PointCloud::Ptr& cloud_msg, int rows, int cols) {
		cloud_msg->width = static_cast<uint32_t>(cols);
		cloud_msg->height = static_cast<uint32_t>(rows);
		cloud_msg->is_dense = static_cast<uint8_t>(false);
		cloud_msg->is_bigendian = static_cast<uint8_t>(false);
		sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
		pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

	}

	PointCloud::Ptr generate_xyz_point_cloud(const cv::Mat& depth_frame,
	                                         const cv::Mat& ideal_pixel_coordinates) {

		const float bad_point = std::numeric_limits<float>::quiet_NaN();
		const float unit_scaling = 0.001f;// millimeters to meters

		PointCloud::Ptr cloud_msg(new PointCloud);
		initialize_xyz_point_cloud(cloud_msg, depth_frame.rows, depth_frame.cols);

		// fill in XYZ values
		sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
		sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
		sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

		cv::MatConstIterator_<cv::Vec2f> xy_it = ideal_pixel_coordinates.begin<cv::Vec2f>();
		cv::MatConstIterator_<uint16_t> depth_it = depth_frame.begin<uint16_t>(), depth_it_end = depth_frame.end<uint16_t>();
		for (; depth_it != depth_it_end; ++iter_x, ++iter_y, ++iter_z, ++xy_it, ++depth_it) {
			uint16_t depth = *depth_it;
			if (depth != 0) {
				float depth_f = static_cast<float> (depth) * unit_scaling;
				cv::Vec2f xy = *xy_it;
				*iter_x = xy[0] * depth_f;
				*iter_y = xy[1] * depth_f;
				*iter_z = depth_f;
			} else {
				*iter_x = *iter_y = *iter_z = bad_point;
			}
		}
		return cloud_msg;
	}

	PointCloud::Ptr generate_xyz_point_cloud(const cv::Mat& depth_frame,
	                                         const cv::Mat& ideal_pixel_coordinates,
	                                         const std::vector<cv::Mat>& dm_maps,
	                                         const std::vector<float>& dm_distances) {

		const float bad_point = std::numeric_limits<float>::quiet_NaN();

		PointCloud::Ptr cloud_msg(new PointCloud);
		initialize_xyz_point_cloud(cloud_msg, depth_frame.rows, depth_frame.cols);

		const float unit_scaling = 0.001f;// millimeters to meters

		// fill in XYZ values
		sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
		sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
		sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

		cv::MatConstIterator_<cv::Vec2f> xy_it = ideal_pixel_coordinates.begin<cv::Vec2f>();

		for (int y = 0; y < depth_frame.rows; y++) {
			const uint16_t* depth_row = depth_frame.ptr<uint16_t>(y);
			for (int x = 0; x < depth_frame.cols; ++xy_it, x++, ++iter_x, ++iter_y, ++iter_z) {
				uint16_t depth = depth_row[x];
				if (depth != 0) {
					float depth_f = static_cast<float> (depth) * unit_scaling;
					depth_f *= get_depth_modifier(depth_f, x, y, dm_maps, dm_distances);
					cv::Vec2f xy = *xy_it;
					*iter_x = xy[0] * depth_f;
					*iter_y = xy[1] * depth_f;
					*iter_z = depth_f;
				} else {
					*iter_x = *iter_y = *iter_z = bad_point;
				}
			}
		}

		return cloud_msg;
	}

	static inline void
	reproject_point_cloud(const PointCloud::Ptr& source_point_cloud, PointCloud::Ptr& reprojected_cloud,
	                      const cv::Mat& rotation, const cv::Mat& translation) {
		double r00 = rotation.at<double>(0, 0), r01 = rotation.at<double>(0, 1), r02 = rotation.at<double>(0, 2);
		double r10 = rotation.at<double>(1, 0), r11 = rotation.at<double>(1, 1), r12 = rotation.at<double>(1, 2);
		double r20 = rotation.at<double>(2, 0), r21 = rotation.at<double>(2, 1), r22 = rotation.at<double>(2, 2);
		double t0 = translation.at<double>(0);
		double t1 = translation.at<double>(1);
		double t2 = translation.at<double>(2);


		sensor_msgs::PointCloud2ConstIterator<float> read_iter_x(*source_point_cloud, "x");
		sensor_msgs::PointCloud2ConstIterator<float> read_iter_y(*source_point_cloud, "y");
		sensor_msgs::PointCloud2ConstIterator<float> read_iter_z(*source_point_cloud, "z");
		sensor_msgs::PointCloud2Iterator<float> iter_x(*reprojected_cloud, "x");
		sensor_msgs::PointCloud2Iterator<float> iter_y(*reprojected_cloud, "y");
		sensor_msgs::PointCloud2Iterator<float> iter_z(*reprojected_cloud, "z");

		for (; read_iter_x !=
		       read_iter_x.end(); ++read_iter_x, ++read_iter_y, ++read_iter_y, ++iter_x, ++iter_y, ++iter_z) {
			float x = *read_iter_x, y = *read_iter_y, z = *read_iter_z;
			*iter_x = static_cast<float> (r00 * x + r01 * y + r02 * z + t0);
			*iter_y = static_cast<float> (r10 * x + r11 * y + r12 * z + t1);
			*iter_z = static_cast<float> (r20 * x + r21 * y + r22 * z + t2);
		}
	}

	PointCloud::Ptr generate_color_registered_point_cloud(
			const PointCloud::Ptr& xyz_point_cloud, const cv::Mat& color_image, const cv::Mat& intrinsic_matrix,
			const cv::Mat& distortion_coefficients, const cv::Mat& rotation_vector, const cv::Mat& rotation_matrix,
			const cv::Mat& translation) {

		PointCloud::Ptr registered_xyzrgb_cloud(new PointCloud);
		int width = color_image.cols, height = color_image.rows;
		registered_xyzrgb_cloud->width = static_cast<uint32_t>(width);
		registered_xyzrgb_cloud->height = static_cast<uint32_t>(height);
		registered_xyzrgb_cloud->is_dense = static_cast<uint8_t>(false);
		registered_xyzrgb_cloud->is_bigendian = static_cast<uint8_t>(false);
		sensor_msgs::PointCloud2Modifier pcd_modifier(*registered_xyzrgb_cloud);
		pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

		//============== SET POINT COLOR ===========================================
		sensor_msgs::PointCloud2Iterator<float> iter_x(*registered_xyzrgb_cloud, "x");
		sensor_msgs::PointCloud2Iterator<float> iter_y(*registered_xyzrgb_cloud, "y");
		sensor_msgs::PointCloud2Iterator<float> iter_z(*registered_xyzrgb_cloud, "z");
		sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*registered_xyzrgb_cloud, "r");
		sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*registered_xyzrgb_cloud, "g");
		sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*registered_xyzrgb_cloud, "b");

		const float bad_point = std::numeric_limits<float>::quiet_NaN();

		cv::MatConstIterator_<cv::Vec3b> color_it = color_image.begin<cv::Vec3b>(), color_it_end = color_image.end<cv::Vec3b>();
		for (; color_it != color_it_end; ++color_it, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
			*iter_x = bad_point;
			*iter_y = bad_point;
			*iter_z = bad_point;
			*iter_b = (*color_it)[0];
			*iter_g = (*color_it)[1];
			*iter_r = (*color_it)[2];
		}

		//==============  SET POINT XYZ POSITIONS =================================
		//register coordinates of points from input cloud by projecting down to the target camera image plane
		int n_points = xyz_point_cloud->width * xyz_point_cloud->height;
		cv::Mat pts3d(1, n_points, CV_32FC3);
		sensor_msgs::PointCloud2ConstIterator<float> read_iter_x(*xyz_point_cloud, "x");
		sensor_msgs::PointCloud2ConstIterator<float> read_iter_y(*xyz_point_cloud, "y");
		sensor_msgs::PointCloud2ConstIterator<float> read_iter_z(*xyz_point_cloud, "z");

		cv::MatIterator_<cv::Vec3f> pts3d_it = pts3d.begin<cv::Vec3f>(),
				pts3d_it_end = pts3d.end<cv::Vec3f>();
		for (; pts3d_it != pts3d_it_end;
		       ++read_iter_x, ++read_iter_y, ++read_iter_z, ++pts3d_it) {

			if (*read_iter_x == bad_point) {
				(*pts3d_it)[0] = 0.0f;
				(*pts3d_it)[1] = 0.0f;
				(*pts3d_it)[2] = 0.0f;
			} else {
				(*pts3d_it)[0] = *read_iter_x;
				(*pts3d_it)[1] = *read_iter_y;
				(*pts3d_it)[2] = *read_iter_z;
			}
		}

		//organize point cloud using the registered coordinates & reproject to new camera's coordinate frame
		double r00 = rotation_matrix.at<double>(0, 0),
				r01 =rotation_matrix.at<double>(0,1), r02 = rotation_matrix.at<double>(0, 2);
		double r10 = rotation_matrix.at<double>(1, 0),
				r11 =rotation_matrix.at<double>(1,1), r12 = rotation_matrix.at<double>(1, 2);
		double r20 = rotation_matrix.at<double>(2, 0),
				r21 =rotation_matrix.at<double>(2,1), r22 = rotation_matrix.at<double>(2, 2);
		double t0 = translation.at<double>(0);
		double t1 = translation.at<double>(1);
		double t2 = translation.at<double>(2);

		cv::Mat pts2d;
		cv::projectPoints(pts3d, rotation_vector, translation, intrinsic_matrix, distortion_coefficients, pts2d);
		cv::MatIterator_<cv::Vec2f> pts2d_it = pts2d.begin<cv::Vec2f>(), pts2d_it_end = pts2d.end<cv::Vec2f>();
		read_iter_x = sensor_msgs::PointCloud2ConstIterator<float>(*xyz_point_cloud, "x");
		read_iter_y = sensor_msgs::PointCloud2ConstIterator<float>(*xyz_point_cloud, "y");
		read_iter_z = sensor_msgs::PointCloud2ConstIterator<float>(*xyz_point_cloud, "z");

		for (; pts2d_it != pts2d_it_end; ++pts2d_it, ++read_iter_x, ++read_iter_y, ++read_iter_z) {
			int col = static_cast<int>(std::round((*pts2d_it)[0]));
			int row = static_cast<int>(std::round((*pts2d_it)[1]));
			if (col >= 0 && col < width && row >= 0 && row < height) {
				float x = *read_iter_x, y = *read_iter_y, z = *read_iter_z;
				float new_pos[3];
				if (x == bad_point) {
					new_pos[2] = new_pos[1] = new_pos[0] = bad_point;
				} else {
					new_pos[0] = static_cast<float> (r00 * x + r01 * y + r02 * z + t0);
					new_pos[1] = static_cast<float> (r10 * x + r11 * y + r12 * z + t1);
					new_pos[2] = static_cast<float> (r20 * x + r21 * y + r22 * z + t2);
				}
				void* point_write_loc = &registered_xyzrgb_cloud->data[row * registered_xyzrgb_cloud->row_step +
				                                                       col * registered_xyzrgb_cloud->point_step];
				memcpy(point_write_loc, new_pos, 3 * sizeof(float));
			}
		}

		return registered_xyzrgb_cloud;
	}

}
