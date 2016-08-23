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
//ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

//rta_openni
#include <rta_openni/point_cloud_generation.h>
#include <rta_openni/point_publishing_node.h>

//std
#include <mutex>

namespace rta_openni {
	using namespace message_filters::sync_policies;

	namespace enc = sensor_msgs::image_encodings;

	class points_node : public point_publishing_node {

		typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
		typedef ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ExactSyncPolicy;
		typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
		typedef message_filters::Synchronizer<ExactSyncPolicy> ExactSynchronizer;

	public:
		points_node() : point_publishing_node(),
		                node_handle(),
		                image_transport(node_handle),
		                xp(node_handle, 68.0,
		                   68.0) {
			int queue_size;
			node_handle.param("queue_size", queue_size, 5);
			bool use_exact_sync;
			node_handle.param("exact_sync", use_exact_sync, false);
			if (!node_handle.getParam("camera_name", camera_name_param)) {
				camera_name_param = "camera";
			}
			if (!node_handle.getParam("namespace", namespace_param)) {
				namespace_param = "remote";
			}
			if (use_exact_sync) {
				exact_synchronizer.reset(
						new ExactSynchronizer(ExactSyncPolicy(queue_size), depth_subscriber, depth_ts_subscriber,
						                      color_subscriber));

				exact_synchronizer->registerCallback(boost::bind(&points_node::image_callback, this, _1, _2, _3));
			} else {
				synchronizer.reset(new Synchronizer(SyncPolicy(queue_size), depth_subscriber, depth_ts_subscriber,
				                                    color_subscriber));
				synchronizer->registerCallback(boost::bind(&points_node::image_callback, this, _1, _2, _3));
			}
			std::lock_guard<std::mutex> lock(connect_mutex);
			ros::SubscriberStatusCallback connect_callback_binding = boost::bind(&points_node::connect_callback, this);
			point_cloud_color_publisher = node_handle.advertise<PointCloud>(
					"/" + namespace_param + "/" + camera_name_param + "/depth_registered/points", 1,
					connect_callback_binding,
					connect_callback_binding);
			point_cloud_publisher = node_handle.advertise<PointCloud>(
					"/" + namespace_param + "/" + camera_name_param + "/depth/points", 1,
					connect_callback_binding,
					connect_callback_binding);
			point_cloud_TS_publisher = node_handle.advertise<PointCloud>(
					"/" + namespace_param + "/" + camera_name_param + "/depth_ts/points", 1,
					connect_callback_binding,
					connect_callback_binding);

		}

		void image_callback(const sensor_msgs::ImageConstPtr& depth_msg,
		                    const sensor_msgs::ImageConstPtr& depth_ts_msg,
		                    const sensor_msgs::ImageConstPtr& color_msg) {
			cv_bridge::CvImageConstPtr depth = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::MONO16);
			cv_bridge::CvImageConstPtr depth_TS = cv_bridge::toCvShare(depth_ts_msg,
			                                                           sensor_msgs::image_encodings::MONO16);
			cv_bridge::CvImageConstPtr color = cv_bridge::toCvShare(color_msg, sensor_msgs::image_encodings::BGR8);

			this->publish_point_clouds(xp,
			                           depth->image, depth_TS->image, color->image, depth_msg, depth_ts_msg, color_msg);
		}

		/**
		 * What to do in case of sibscribe /
		 */
		void connect_callback() {
			std::lock_guard<std::mutex> lock(connect_mutex);
			if (point_cloud_publisher.getNumSubscribers() +
			    point_cloud_TS_publisher.getNumSubscribers() +
			    point_cloud_color_publisher.getNumSubscribers() == 0) {
				depth_subscriber.unsubscribe();
				depth_ts_subscriber.unsubscribe();
				color_subscriber.unsubscribe();
			} else if (!depth_subscriber.getSubscriber()) {
				depth_subscriber.subscribe(image_transport, "/" + this->camera_name_param + "/depth/image", 1);
				depth_ts_subscriber.subscribe(image_transport, "/" + this->camera_name_param + "/depth_ts/image", 1);
				color_subscriber.subscribe(image_transport, "/" + this->camera_name_param + "/rgb/image", 1);
			}
		}

	private:
		//params
		std::string camera_name_param;
		std::string namespace_param;

		//handles
		ros::NodeHandle node_handle;
		image_transport::ImageTransport image_transport;

		//=======point cloud publishing=======
		std::mutex connect_mutex;

		//camera parameters
		xtion_parameters xp;

		//synchronization
		image_transport::SubscriberFilter depth_subscriber, depth_ts_subscriber, color_subscriber;
		std::shared_ptr<Synchronizer> synchronizer;
		std::shared_ptr<ExactSynchronizer> exact_synchronizer;

	};
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "points");
	rta_openni::points_node node;
	ros::spin();
	return 0;
}