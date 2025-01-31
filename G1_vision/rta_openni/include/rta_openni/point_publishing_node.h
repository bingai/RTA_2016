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

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <rta_openni/xtion_parameters.h>

#pragma once

namespace rta_openni {
	class point_publishing_node {
	public:
		point_publishing_node();

		point_publishing_node(std::string camera_name);

		virtual ~point_publishing_node();

	protected:
		ros::NodeHandle node_handle;

		void publish_point_clouds(const xtion_parameters& xp,
		                          const cv::Mat& depth_frame,
		                          const cv::Mat& time_smoothed_depth_frame,
		                          const cv::Mat& color_frame,
		                          const sensor_msgs::ImageConstPtr& depth_msg,
		                          const sensor_msgs::ImageConstPtr& depth_ts_msg,
		                          const sensor_msgs::ImageConstPtr& color_msg);

		//=======point cloud publishing=======
		ros::Publisher point_cloud_publisher;
		ros::Publisher point_cloud_TS_publisher;
		ros::Publisher point_cloud_color_publisher;


	};
}


