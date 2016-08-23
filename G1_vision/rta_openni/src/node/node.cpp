//  ================================================================
//  Created by Gregory Kramida on 6/20/16.
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
#include <rta_openni/point_cloud_generation.h>
#include <rta_openni/xtion_parameters.h>
#include <rta_openni/openni_capture.h>
#include <rta_openni/point_publishing_node.h>
#include <rta_openni/UseHighResolutionColor.h>
#include <rta_openni/GetCameraIntrinsics.h>
#include <rta_openni/GetCameraExtrinsics.h>
#include <rta_openni/SetIrStream.h>
#include <rta_openni/SetEmitterState.h>
#include <rta_openni/GetCurrentFrames.h>

//ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/distortion_models.h>
#include <camera_info_manager/camera_info_manager.h>

//stdlib
#include <mutex>
#include <chrono>

//OpenCV
#include <opencv2/calib3d/calib3d.hpp>

typedef sensor_msgs::PointCloud2 PointCloud;

namespace rta_openni {

	class node : public point_publishing_node {

		//TODO rewrite the class to properly use the camera_publisher class from the image_transport ROS package instead of custom publisher code - issue 10
		//TODO add subscriber callbacks for color, ir, depth images the way it's done in openni2_camera package openni2_driver - issue 10
	public:


		inline static void
		convert_cv_to_tf(tf::Transform& transform, const cv::Mat& rotation, const cv::Mat& translation) {

			tf::Matrix3x3 rotmat;
			rotmat.setValue(rotation.at<double>(0, 0), rotation.at<double>(0, 1),
			                rotation.at<double>(0, 2),
			                rotation.at<double>(1, 0), rotation.at<double>(1, 1),
			                rotation.at<double>(1, 2),
			                rotation.at<double>(2, 0), rotation.at<double>(2, 1),
			                rotation.at<double>(2, 2));
			tf::Quaternion base_to_color_rotation;
			cv::Mat color_rotation_vec;
			rotmat.getRotation(base_to_color_rotation);
			cv::Rodrigues(rotation, color_rotation_vec);
			transform.setRotation(base_to_color_rotation);
			transform.setOrigin(tf::Vector3(translation.at<double>(0), translation.at<double>(1),
			                                translation.at<double>(2)));
		}

		inline static void
		convert_tf_to_cv
				(const tf::Transform& transform, cv::Mat& rotation, cv::Mat& translation) {

			tf::Matrix3x3 rotmat;
			rotmat.setRotation(transform.getRotation());

			rotation = cv::Mat(3, 3, CV_64F);
			translation = cv::Mat(3, 1, CV_64F);

			rotation.at<double>(0, 0) = rotmat.getRow(0).getX();
			rotation.at<double>(0, 1) = rotmat.getRow(0).getY();
			rotation.at<double>(0, 2) = rotmat.getRow(0).getZ();
			rotation.at<double>(1, 0) = rotmat.getRow(1).getX();
			rotation.at<double>(1, 1) = rotmat.getRow(1).getY();
			rotation.at<double>(1, 2) = rotmat.getRow(1).getZ();
			rotation.at<double>(2, 0) = rotmat.getRow(2).getX();
			rotation.at<double>(2, 1) = rotmat.getRow(2).getY();
			rotation.at<double>(2, 2) = rotmat.getRow(2).getZ();

			tf::Vector3 transl = transform.getOrigin();
			translation.at<double>(0) = transl.getX();
			translation.at<double>(1) = transl.getY();
			translation.at<double>(1) = transl.getZ();
		}

		inline static void convert_to_camera_info(sensor_msgs::CameraInfo& info,
		                                          const unsigned int width, const unsigned int height,
		                                          const cv::Mat& intrinsic_mat,
		                                          const cv::Mat& distortion_coefficients) {
			info.width = width;
			info.height = height;
			std::memcpy(info.K.data(), intrinsic_mat.data,
			            info.K.size() * sizeof(double));
			info.D.resize(5);
			std::memcpy(info.D.data(), distortion_coefficients.data,
			            info.D.size() * sizeof(double));
			info.P.fill(0.0);
			std::memcpy(info.P.data(), info.K.data(),
			            3 * sizeof(double));
			std::memcpy(info.P.data() + 4, info.K.data() + 3,
			            3 * sizeof(double));
			std::memcpy(info.P.data() + 8, info.K.data() + 6,
			            3 * sizeof(double));
		}

		node() : point_publishing_node("camera"),
				capture(false), mutex(),
				ir_frame(cv::Size(openni2_xtion_capture::lo_ir_width, openni2_xtion_capture::lo_ir_height), CV_16UC1),
				color_frame(cv::Size(openni2_xtion_capture::lo_color_width,
				                     openni2_xtion_capture::lo_color_height), CV_8UC3),
				depth_frame(cv::Size(openni2_xtion_capture::depth_width,
				                     openni2_xtion_capture::depth_height), CV_16UC1),
				time_smoothed_depth_frame(cv::Size(openni2_xtion_capture::depth_width,
				                                  openni2_xtion_capture::depth_height), CV_16UC1),
				ir_camera_info(),
				color_camera_info(),
				depth_camera_info(),
				time_stamp(),
				xp(node_handle, capture.get_depth_horizontal_field_of_view(),
				   capture.get_color_horizontal_field_of_view()),
				// =============================== INITIALIZE PUBLISHERS AND SERVICES ==================================
				image_transport(node_handle),
				ir_publisher(image_transport.advertise("/camera/ir/image", 1)),
				color_publisher(image_transport.advertise("/camera/rgb/image", 1)),
				depth_publisher(image_transport.advertise("/camera/depth/image", 1)),
				depth_TS_publisher(image_transport.advertise("/camera/depth_ts/image", 1)),
				ir_camera_info_publisher(node_handle.advertise<sensor_msgs::CameraInfo>("/camera/ir/camera_info", 1)),
				color_camera_info_publisher(
						node_handle.advertise<sensor_msgs::CameraInfo>("/camera/rgb/camera_info", 1)),
				depth_camera_info_publisher(
						node_handle.advertise<sensor_msgs::CameraInfo>("/camera/depth/camera_info", 1)),

				color_mode_service(node_handle.advertiseService("/camera/use_high_resolution_color",
				                                                &node::set_use_high_resolution_color, this)),
				intrinsics_service(node_handle.advertiseService("/camera/get_camera_intrinsics",
				                                                &node::get_camera_intrinsics, this)),
				extrinsics_service(node_handle.advertiseService("/camera/get_camera_extrinsics",
				                                                &node::get_camera_extrinsics, this)),
				get_current_frames_service(
						node_handle.advertiseService("/camera/get_current_frames", &node::get_current_frames, this)),
				ir_mode_service(
						node_handle.advertiseService("/camera/set_ir_stream", &node::set_ir_stream, this)),
				emitter_state_service(
						node_handle.advertiseService("/camera/set_emitter_state", &node::set_emitter_state, this)),
				camera_name("camera"),
				tf_prefix(""),
				housing("/left_hand_camera") {
			// ============================== PARSE LAUNCH FILE PARAMS =================================================
			if (!node_handle.getParam("camera_name", camera_name)) {
				ROS_INFO("camera_name parameter undefined. Setting to \"camera\".");
			}
			if (!node_handle.getParam("tf_prefix", tf_prefix)) {
				ROS_INFO("tf_prefix parameter undefined. Setting to \"\".");
			}
			if (!node_handle.getParam("housing", housing)) {
				ROS_INFO("housing parameter undefined. Setting to \"/left_hand_camera\".");
			}

			// =============================== INITIALIZE CAMERA PARAMS ================================================
			const unsigned int depth_width = openni2_xtion_capture::depth_width;
			const unsigned int depth_height = openni2_xtion_capture::depth_height;
			const unsigned int lo_color_width = openni2_xtion_capture::lo_color_width;
			const unsigned int lo_color_height = openni2_xtion_capture::lo_color_height;
			const unsigned int lo_ir_width = openni2_xtion_capture::lo_ir_width;
			const unsigned int lo_ir_height = openni2_xtion_capture::lo_ir_height;

			//define depth to color transform
			convert_cv_to_tf(depth_to_color, xp.color_rotation_matrix, xp.color_translation);
			//invert (calibration gives transform from camera a space to other camera b's space, not from camera a to camera b)
			depth_to_color = depth_to_color.inverse();

			//================== READ HAND TO DEPTH CAMERA CALIBRATION =================================================
			cv::Mat depth_rotation;
			cv::Mat depth_translation;
			bool calibration_file_found = false;
			std::string calibration_file_path;
			if (node_handle.getParam("ir_arm_calibration_file_path", calibration_file_path)) {
				cv::FileStorage fs(calibration_file_path, cv::FileStorage::READ);
				if (fs.isOpened()) {
					calibration_file_found = true;
					cv::FileNode stereo_calib_node = fs["Rig"];
					cv::FileNode cameras_node = stereo_calib_node["Cameras"];
					cv::FileNode camera_1_node = cameras_node[1];
					cv::FileNode extrinsics_node = camera_1_node["Extrinsics"];
					extrinsics_node["rotation"] >> depth_rotation;
					extrinsics_node["translation"] >> depth_translation;
				} else {
					ROS_INFO("Calibration file not found at '%s'. Using defaults.", calibration_file_path.c_str());
				}
			} else {
				ROS_INFO("The calibration_file_path parameter was not specified. Using defaults.");
			}
			if (!calibration_file_found) {
				depth_translation = (cv::Mat_<double>(3, 1) << 0.03893926, 0.12502476, 0.04734136);
				depth_rotation = cv::Mat::eye(3, 3, CV_64F);
			}
			convert_cv_to_tf(left_hand_camera_to_depth, depth_rotation, depth_translation);

			//========================CameraInfo generation=============================================================
			convert_to_camera_info(ir_camera_info, lo_ir_width, lo_ir_height,
			                       xp.lo_ir_camera_matrix, xp.depth_distortion_coefficients);
			convert_to_camera_info(color_camera_info, lo_color_width, lo_color_height,
			                       xp.lo_color_camera_matrix, xp.color_distortion_coefficients);
			convert_to_camera_info(depth_camera_info, depth_width, depth_height,
			                       xp.depth_camera_matrix, xp.depth_distortion_coefficients);

			//======================== define frames ===================================================================
			optical_color_frame_id = tf_prefix + "/" + camera_name + "_color_optical_frame";
			optical_depth_frame_id = tf_prefix + "/" + camera_name + "_depth_optical_frame";
		};
//================================== SERVICE HANDLERS ==================================================================

		bool set_use_high_resolution_color(UseHighResolutionColor::Request& request,
		                                   UseHighResolutionColor::Response& response) {
			std::lock_guard<std::mutex> lock(mutex);
			color_frame = cv::Mat(cv::Size(
					request.use_high_resolution_color ? rta_openni::openni2_xtion_capture::hi_color_width
					                                  : rta_openni::openni2_xtion_capture::lo_color_width,
					request.use_high_resolution_color ? rta_openni::openni2_xtion_capture::hi_color_height
					                                  : rta_openni::openni2_xtion_capture::lo_color_height), CV_8UC3);
			capture.set_use_high_resolution_color(request.use_high_resolution_color);
			color_camera_info.width = static_cast<unsigned int>(color_frame.cols);
			color_camera_info.height = static_cast<unsigned int>(color_frame.rows);
			response.success = static_cast<uint8_t>(true);
			return true;
		}

		bool set_ir_stream(SetIrStream::Request& request,
		                   SetIrStream::Response& response) {
			std::lock_guard<std::mutex> lock(mutex);
			bool turn_off_emitter = !capture.emitter_is_on();
			this->capture.set_ir_stream_state(request.on);
			if (turn_off_emitter) {
				wait_for_some_time(0.2);
				capture.set_emitter_state(false);
			}
			response.success = static_cast<uint8_t>(true);
			return true;
		}

		bool set_emitter_state(SetEmitterState::Request& request,
		                       SetEmitterState::Response& response) {
			std::lock_guard<std::mutex> lock(mutex);
			response.success = static_cast<uint8_t>(true);
			if (request.state == "on") {
				this->capture.set_emitter_state(true);
			} else if (request.state == "off") {
				this->capture.set_emitter_state(false);
			} else if (request.state == "toggle") {
				this->capture.toggle_emitter_state();
			} else {
				response.success = static_cast<uint8_t>(false);
			}
			return true;
		}

		bool get_camera_intrinsics(GetCameraIntrinsics::Request& request,
		                           GetCameraIntrinsics::Response& response) {
			const std::string depth_literal = "Depth";
			const std::string lo_color_literal = "LowResolutionColor";
			const std::string hi_color_literal = "HighResolutionColor";
			const std::string lo_ir_literal = "IR";
			const std::string hi_ir_literal = "HighResolutionIR";
			std::array<std::string, 5> all_literals{depth_literal, lo_color_literal, hi_color_literal, lo_ir_literal};
			response.success = static_cast<uint8_t>(true);
			if (request.which_camera == depth_literal) {
				std::memcpy(response.camera_matrix_array.data(), xp.depth_camera_matrix.data,
				            response.camera_matrix_array.size() * sizeof(double));
				std::memcpy(response.distortion_coefficient_array.data(), xp.depth_distortion_coefficients.data,
				            response.distortion_coefficient_array.size() * sizeof(double));
			} else if (request.which_camera == lo_color_literal) {
				std::memcpy(response.camera_matrix_array.data(), xp.lo_color_camera_matrix.data,
				            response.camera_matrix_array.size() * sizeof(double));
				std::memcpy(response.distortion_coefficient_array.data(), xp.color_distortion_coefficients.data,
				            response.distortion_coefficient_array.size() * sizeof(double));
			} else if (request.which_camera == lo_ir_literal) {
				std::memcpy(response.camera_matrix_array.data(), xp.lo_ir_camera_matrix.data,
				            response.camera_matrix_array.size() * sizeof(double));
				std::memcpy(response.distortion_coefficient_array.data(), xp.depth_distortion_coefficients.data,
				            response.distortion_coefficient_array.size() * sizeof(double));
			} else if (request.which_camera == hi_color_literal) {
				response.message = "Support for " + request.which_camera + " not yet implemented.";
				response.success = static_cast<uint8_t>(false);
			} else {
				std::stringstream ss;
				ss << "Error: which_camera parameter must be set to one of [";
				for (std::string literal : all_literals) {
					ss << literal;
					if (literal != all_literals.back()) {
						ss << ", ";
					}
				}
				ss << "].";
				response.message = ss.str();
				response.success = static_cast<uint8_t>(false);
			}
			return true;
		}

		bool get_camera_extrinsics(GetCameraExtrinsics::Request& request,
		                           GetCameraExtrinsics::Response& response) {
			std::memcpy(response.color_rotation.data(), xp.color_rotation_matrix.data,
			            response.color_rotation.size() * sizeof(double));
			std::memcpy(response.color_translation.data(), xp.color_translation.data,
			            response.color_translation.size() * sizeof(double));
			return true;
		}

//=============== GET CURRENT FRAME SERVICE HANDLER & HARDWARE CHANGE HELPERS ==========================================
		inline
		void wait_for_some_time(double delay) {
			auto t_start = std::chrono::high_resolution_clock::now();
			std::chrono::high_resolution_clock::time_point t_end;
			do {
				t_end = std::chrono::high_resolution_clock::now();
				this->read();
				this->publish();
			} while (std::chrono::duration<double>(t_end - t_start).count() < (double) delay);
		}

		inline
		void change_state_safely(bool emitter_on, bool ir_stream_on) {
			if (capture.emitter_is_on() != emitter_on || capture.ir_stream_is_on() != ir_stream_on) {
				if (capture.emitter_is_on() != emitter_on) {
					capture.set_emitter_state(emitter_on);
				}
				if (capture.ir_stream_is_on() != ir_stream_on) {
					capture.set_ir_stream_state(ir_stream_on);
				}
				wait_for_some_time(1.0);
			}
		}

		bool get_current_frames(GetCurrentFrames::Request& request,
		                        GetCurrentFrames::Response& response) {
			std::lock_guard<std::mutex> lock(mutex);
			bool emitter_was_on = capture.emitter_is_on();
			bool ir_was_on = capture.ir_stream_is_on();
			//set up to appropriate state (color on, emitter on)
			change_state_safely(true, false);

			capture.unsafe_read(color_frame, depth_frame, time_smoothed_depth_frame, time_stamp);
			this->publish();

			cv_bridge::CvImage(std_msgs::Header(), "bgr8",
			                   color_frame).toImageMsg(response.color);
			cv_bridge::CvImage(std_msgs::Header(), "mono16",
			                   depth_frame).toImageMsg(response.depth);

			capture.set_ir_stream_state(true);
			capture.unsafe_read_ir(ir_frame, depth_frame, time_smoothed_depth_frame, time_stamp);
			capture.set_emitter_state(false);
			wait_for_some_time(0.2);

			capture.unsafe_read_ir(ir_frame, depth_frame, time_smoothed_depth_frame, time_stamp);
			this->publish();
			cv_bridge::CvImage(std_msgs::Header(), "mono16",
			                   ir_frame).toImageMsg(response.ir);

			response.color.header.frame_id = optical_color_frame_id;
			response.color.header.stamp = time_stamp;
			response.depth.header.frame_id = optical_depth_frame_id;
			response.depth.header.stamp = time_stamp;
			response.ir.header = response.depth.header;
			response.success = true;

			change_state_safely(emitter_was_on, ir_was_on);

			return true;
		}

		void read_and_publish_thread_safe() {
			std::lock_guard<std::mutex> lock(mutex);
			this->read();
			this->publish();
		}

		~node() {};

	private:

//============================== PRIVATE VARIABLES =====================================================================

		rta_openni::openni2_xtion_capture capture;
		std::mutex mutex;

		//==== local frame storage ====
		cv::Mat ir_frame;
		cv::Mat color_frame;
		cv::Mat depth_frame;
		cv::Mat time_smoothed_depth_frame;

		//==== local camera info storage ===
		sensor_msgs::CameraInfo ir_camera_info;
		sensor_msgs::CameraInfo color_camera_info;
		sensor_msgs::CameraInfo depth_camera_info;


		//for keeping track of time at each frame capture
		ros::Time time_stamp;

		//camera parameters
		xtion_parameters xp;



		//=======image publishing=======
		image_transport::ImageTransport image_transport;
		image_transport::Publisher ir_publisher;
		image_transport::Publisher color_publisher;
		image_transport::Publisher depth_publisher;
		image_transport::Publisher depth_TS_publisher;

		//=======camera info publishing===========
		ros::Publisher ir_camera_info_publisher;
		ros::Publisher color_camera_info_publisher;
		ros::Publisher depth_camera_info_publisher;

		//=======service broadcasting=========
		ros::ServiceServer color_mode_service;
		ros::ServiceServer intrinsics_service;
		ros::ServiceServer extrinsics_service;
		ros::ServiceServer ir_mode_service;
		ros::ServiceServer emitter_state_service;
		ros::ServiceServer get_current_frames_service;

		//=======transform broadcasting======
		tf::TransformBroadcaster tf_broadcaster;
		tf::Transform depth_to_color;
		tf::Transform left_hand_camera_to_depth;

		//=======storage of launch-file parameters
		std::string camera_name;
		std::string tf_prefix;
		std::string housing;
		std::string optical_depth_frame_id;
		std::string optical_color_frame_id;


//================================= PRIVATE METHODS ====================================================================

		void read() {
			if (capture.ir_stream_is_on()) {
				capture.unsafe_read_ir(ir_frame, depth_frame, time_smoothed_depth_frame, time_stamp);
			} else {
				capture.unsafe_read(color_frame, depth_frame, time_smoothed_depth_frame, time_stamp);
			}
		}

		void publish() {
			// *** broadcast the color transform ***
			tf_broadcaster.sendTransform(
					tf::StampedTransform(left_hand_camera_to_depth, ros::Time::now(), housing,
					                     tf_prefix + "/" + camera_name + "_link"));
			tf_broadcaster.sendTransform(
					tf::StampedTransform(depth_to_color, ros::Time::now(),
					                     tf_prefix + "/" + camera_name + "_depth_frame",
					                     tf_prefix + "/" + camera_name + "_color_frame"));

			// *** send frame image messages ***
			sensor_msgs::ImagePtr color_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",
			                                                     this->color_frame).toImageMsg();
			sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16",
			                                                     this->depth_frame).toImageMsg();
			sensor_msgs::ImagePtr depth_TS_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16",
			                                                        this->time_smoothed_depth_frame).toImageMsg();


			color_msg->header.frame_id = optical_color_frame_id;
			color_msg->header.stamp = time_stamp;
			depth_msg->header.frame_id = optical_depth_frame_id;
			depth_msg->header.stamp = time_stamp;
			depth_TS_msg->header = depth_msg->header;

			if (this->capture.ir_stream_is_on()) {
				sensor_msgs::ImagePtr ir_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16",
				                                                  this->ir_frame).toImageMsg();
				ir_msg->header = depth_msg->header;
				ir_publisher.publish(ir_msg);
			}

			color_publisher.publish(color_msg);
			depth_publisher.publish(depth_msg);
			depth_TS_publisher.publish(depth_TS_msg);

			//*** publish camera_info messages
			ir_camera_info.header = depth_msg->header;
			depth_camera_info.header = depth_msg->header;
			color_camera_info.header = color_msg->header;

			ir_camera_info_publisher.publish(ir_camera_info);
			depth_camera_info_publisher.publish(depth_camera_info);
			color_camera_info_publisher.publish(color_camera_info);

			publish_point_clouds(xp,
			                     depth_frame,
			                     time_smoothed_depth_frame,
			                     color_frame,
			                     depth_msg,
			                     depth_TS_msg,
			                     color_msg);
		}

	};
}


int main(int argc, char** argv) {
	// initialize ROS with command-line arguments,
	ros::init(argc, argv, "camera");
	rta_openni::node node;
	ros::Rate loop_rate(30);//hz
	while (ros::ok()) {
		node.read_and_publish_thread_safe();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}