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


#include <rta_openni/point_publishing_node.h>
#include <rta_openni/point_cloud_generation.h>


namespace rta_openni {
	point_publishing_node::point_publishing_node() : node_handle() {}

	point_publishing_node::point_publishing_node(std::string camera_name) :
			node_handle(),
			point_cloud_publisher(node_handle.advertise<PointCloud>("/" + camera_name + "/depth/points", 1)),
			point_cloud_TS_publisher(node_handle.advertise<PointCloud>("/" + camera_name + "/depth_ts/points", 1)),
			point_cloud_color_publisher(
					node_handle.advertise<PointCloud>("/" + camera_name + "/depth_registered/points", 1)) {}

	point_publishing_node::~point_publishing_node() {}

	void point_publishing_node::publish_point_clouds(const xtion_parameters& xp,
	                                                 const cv::Mat& depth_frame,
	                                                 const cv::Mat& time_smoothed_depth_frame,
	                                                 const cv::Mat& color_frame,
	                                                 const sensor_msgs::ImageConstPtr& depth_msg,
	                                                 const sensor_msgs::ImageConstPtr& depth_TS_msg,
	                                                 const sensor_msgs::ImageConstPtr& color_msg) {
		// *** generate & send point cloud & time-smoothed point cloud messages ***

		PointCloud::Ptr cloud_msg, cloud_TS_msg;

		if (xp.displacement_maps_loaded) {
			// correct depth using lookups on-the-fly
			cloud_msg = rta_openni::generate_xyz_point_cloud(depth_frame, xp.depth_ideal_pixel_coordinates,
			                                                 xp.displacement_maps, xp.displacement_map_distances);
			cloud_TS_msg = rta_openni::generate_xyz_point_cloud(time_smoothed_depth_frame,
			                                                    xp.depth_ideal_pixel_coordinates,
			                                                    xp.displacement_maps, xp.displacement_map_distances);
		} else {
			cloud_msg = rta_openni::generate_xyz_point_cloud(depth_frame, xp.depth_ideal_pixel_coordinates);
			cloud_TS_msg = rta_openni::generate_xyz_point_cloud(time_smoothed_depth_frame,
			                                                    xp.depth_ideal_pixel_coordinates);
		}

		cloud_msg->header = depth_msg->header;
		cloud_TS_msg->header = depth_TS_msg->header;

		PointCloud::Ptr cloud_color_msg =
				generate_color_registered_point_cloud(cloud_msg, color_frame, xp.lo_color_camera_matrix,
				                                      xp.color_distortion_coefficients, xp.color_rotation_vector,
				                                      xp.color_rotation_matrix, xp.color_translation);


		cloud_color_msg->header = color_msg->header;
		point_cloud_publisher.publish(cloud_msg);
		point_cloud_TS_publisher.publish(cloud_TS_msg);
		point_cloud_color_publisher.publish(cloud_color_msg);
	}
}