//  ================================================================
//  Created by Gregory Kramida on 8/1/16.
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
#include <rta_openni/SetIrStream.h>
#include <rta_openni/SetEmitterState.h>
#include <rta_openni/GetCurrentFrames.h>
#include <signal.h>

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

//OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

//stdlib & boost
#include <mutex>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>


#ifdef X11_FOUND

#include <X11/Xlib.h>

#endif

namespace rta_openni {
	class calibration_capture_node {
	public:
		calibration_capture_node() :
				node_handle(),
				image_transport(node_handle),
				depth_subscriber(
						image_transport.subscribe("/camera/depth/image", 1, &calibration_capture_node::depth_callback,
						                          this)),
				color_subscriber(
						image_transport.subscribe("/camera/rgb/image", 1, &calibration_capture_node::color_callback,
						                          this)),
				ir_subscriber(
						image_transport.subscribe("/camera/ir/image", 1, &calibration_capture_node::ir_callback, this)),
				left_hand_camera_subscriber(
						image_transport.subscribe("/cameras/left_hand_camera/image", 1, &calibration_capture_node::left_hand_camera_callback, this)),
				emitter_state_client(node_handle.serviceClient<rta_openni::SetEmitterState>(
						"/camera/set_emitter_state")),
				ir_stream_client(node_handle.serviceClient<rta_openni::SetIrStream>(
						"/camera/set_ir_stream")),
				get_current_frames_client(
						node_handle.serviceClient<rta_openni::GetCurrentFrames>("/camera/get_current_frames")),
				color_window_name("Color"),
				depth_window_name("Depth"),
				ir_window_name("IR"),
				lh_camera_window_name("Hand_Camera"),
				draw_chessboard(false),
				corner_refinement(false),
				chessboard_size(cv::Size(14, 9)),
				current_frame_ix(0),
				output_folder(""),
				lh_cam_protection(){

			if (!node_handle.getParam("output_folder", output_folder)) {
				ROS_INFO("output_folder parameter undefined. Setting to \"%s\".", output_folder.c_str());
			}else{
				ROS_INFO("Will save images to: \"%s\".", output_folder.c_str());
			}

			if (!node_handle.getParam("start_at", current_frame_ix)) {
				ROS_INFO("start_at parameter undefined. Setting to %d.",current_frame_ix);
			}
			instance = this;

#ifdef X11_FOUND
			Display* disp = XOpenDisplay(NULL);
			Screen* scrn = DefaultScreenOfDisplay(disp);
			const int screen_width = scrn->width;
			const int screen_height = scrn->height;
#else
			const int screen_width = 1920;
			const int screen_height = 1200;
#endif
			const int separation = 2;
			const int right_offset = 70;//approx launcher width
			const int top_offset = 30;
			const int window_header_height = 28;
			const int horiz_limit = screen_width - right_offset;
			const int vert_limit = screen_height - top_offset;

			const int frame_width = 640, frame_height = 480;
			const int tiles_x = 2, tiles_y = 2;

			//how much to scale down the images
			float factor_x = (float) (horiz_limit - (separation * (tiles_x - 1))) / (float) (frame_width * tiles_x);
			float factor_y = (float) (vert_limit - ((separation + window_header_height) * (tiles_y - 1))) /
			                 (float) (frame_height * tiles_y);
			float factor = std::min(factor_x, factor_y);

			int window_width = frame_width * factor, window_height = frame_height * factor - window_header_height;

			const char* window_names[4] = {color_window_name.c_str(), depth_window_name.c_str(), ir_window_name.c_str(),
			                               lh_camera_window_name.c_str()};
			std::string path = ros::package::getPath("rta_openni");
			path = path + "/assets/catffaceheader.jpg";

			int y = top_offset;
			int i_window = 0;
			for (int y_window = 0;
			     y_window < tiles_y; y_window++, y += (window_height + separation + window_header_height)) {
				int x = right_offset;
				for (int x_window = 0; x_window < tiles_x; x_window++, i_window++, x += (window_width + separation)) {
					cv::namedWindow(window_names[i_window], cv::WINDOW_NORMAL);
					cv::moveWindow(window_names[i_window], x, y);
					cv::imshow(window_names[i_window], cv::imread(path.c_str()));
					cv::resizeWindow(window_names[i_window], window_width, window_height);
				}
			}

		}

		void reset_openni_node() {
			ROS_INFO("Turning the IR stream off & making sure the IR emitter is on.");
			turn_off_ir_stream();
			turn_on_emitter();
		}

		static void handle_sigterm(int sig) {
			if (instance) {
				instance->reset_openni_node();
			}
			ros::shutdown();
		}

		void turn_on_ir_stream() {
			ir_stream_service.request.on = true;
			if (!ir_stream_client.call(ir_stream_service)) {
				ROS_WARN("Call to custom openni node to turn on IR stream failed.");
			}
		}

		void turn_off_ir_stream() {
			ir_stream_service.request.on = false;
			if (!ir_stream_client.call(ir_stream_service)) {
				ROS_WARN("Call to custom openni node to turn off IR stream failed.");
			}
		}

		void toggle_ir_stream() {
			ir_stream_service.request.on = !ir_stream_service.request.on;
			if (!ir_stream_client.call(ir_stream_service)) {
				ROS_WARN("Call to custom openni node to toggle IR stream failed.");
			}
		}

		void turn_on_emitter() {
			emitter_state_service.request.state = "on";
			if (!emitter_state_client.call(emitter_state_service)) {
				ROS_WARN("Call to custom openni node to turn on OpenNI emitter failed.");
			}
		}

		void toggle_emitter() {
			emitter_state_service.request.state = "toggle";
			if (!emitter_state_client.call(emitter_state_service)) {
				ROS_WARN("Call to custom openni node to toggle OpenNI emitter failed.");
			}
		}

		inline std::string make_save_path(std::string postfix){
			boost::filesystem::path of(output_folder);
			std::stringstream ss;
			ss << "capture_" << std::setw(3) << std::setfill('0') << current_frame_ix << "_" << postfix << ".png";
			boost::filesystem::path file(ss.str());
			boost::filesystem::path full_path = of / file;
			return full_path.string();
		}


		int run() {
			ros::Rate loop_rate(30);//hz
			bool quit = false;
			ROS_INFO("|----------------------------------------------------|");
			ROS_INFO("|--------------- CONTROLS ---------------------------|");
			ROS_INFO("|----------------------------------------------------|");
			ROS_INFO("| q or ESC |  quit                                   |");
			ROS_INFO("| s        |  save images                            |");
			ROS_INFO("| i        |  toggle ir/color                        |");
			ROS_INFO("| e        |  toggle emitter                         |");
			ROS_INFO("| c        |  toggle corner detection & display      |");
			ROS_INFO("| r        |  toggle corner refinement               |");
			ROS_INFO("|----------------------------------------------------|");
			while (ros::ok() && !quit) {
				ros::spinOnce();
				loop_rate.sleep();
				char key = cv::waitKey(1);
				if (key > 0) {
					switch (key) {
						case 's':
							if (get_current_frames_client.call(get_current_frames_service)) {
								cv::Mat arm_frame;
								{
									std::lock_guard<std::mutex> lock(lh_cam_protection);
									last_lh_cam_frame.copyTo(arm_frame);
								}
								cv_bridge::CvImagePtr color = cv_bridge::toCvCopy(
										boost::shared_ptr<sensor_msgs::Image>(boost::shared_ptr<sensor_msgs::Image>{},
										                                      &get_current_frames_service.response.color),
										sensor_msgs::image_encodings::BGR8);
								cv_bridge::CvImagePtr depth = cv_bridge::toCvCopy(
										boost::shared_ptr<sensor_msgs::Image>(boost::shared_ptr<sensor_msgs::Image>{},
										                                      &get_current_frames_service.response.depth),
										sensor_msgs::image_encodings::MONO16);
								cv_bridge::CvImagePtr ir = cv_bridge::toCvCopy(
										boost::shared_ptr<sensor_msgs::Image>(boost::shared_ptr<sensor_msgs::Image>{},
										                                      &get_current_frames_service.response.ir),
										sensor_msgs::image_encodings::MONO16);

								cv::imwrite(make_save_path("rgb"), color->image);
								cv::imwrite(make_save_path("depth"),depth->image);
								cv::imwrite(make_save_path("ir"),ir->image);
								cv::imwrite(make_save_path("arm"),arm_frame);
								current_frame_ix++;
							} else {
								ROS_WARN("Call to custom openni node to get current frames failed.");
							}
							break;
						case 27:
						case 'q':
							quit = true;
							break;
						case 'i':
							toggle_ir_stream();
							break;
						case 'e':
							toggle_emitter();
							break;
						case 'c':
							draw_chessboard = !draw_chessboard;
							break;
						case 'r':
							corner_refinement = !corner_refinement;
							break;
						default:
							break;
					}
				}
			}
			reset_openni_node();
			return 0;
		}

	private:
		//====ROS stuff====
		ros::NodeHandle node_handle;
		// subscribers
		image_transport::ImageTransport image_transport;
		image_transport::Subscriber color_subscriber;
		image_transport::Subscriber depth_subscriber;
		image_transport::Subscriber ir_subscriber;
		image_transport::Subscriber left_hand_camera_subscriber;

		//services
		rta_openni::SetEmitterState emitter_state_service;
		ros::ServiceClient emitter_state_client;
		rta_openni::SetIrStream ir_stream_service;
		ros::ServiceClient ir_stream_client;
		rta_openni::GetCurrentFrames get_current_frames_service;
		ros::ServiceClient get_current_frames_client;

		//calibration board stuff
		static calibration_capture_node* instance;
		bool draw_chessboard;
		bool corner_refinement;
		cv::Size chessboard_size;
		static cv::TermCriteria corner_refinement_criteria;
		static cv::Size corner_window_size;
		static cv::Size corner_zero_zone;

		//output
		int current_frame_ix;
		std::string output_folder;

		std::mutex lh_cam_protection;
		cv::Mat last_lh_cam_frame;


		inline
		void draw_chessboard_helper(cv::Mat& frame) {
			std::vector<cv::Point2f> corners;
			bool patternfound = cv::findChessboardCorners(frame, chessboard_size, corners);
			if (corner_refinement && patternfound) {
				if (frame.channels() == 3) {
					cv::Mat grey_frame;
					cv::cvtColor(frame, grey_frame, cv::COLOR_BGR2GRAY);
					cv::cornerSubPix(grey_frame, corners, corner_window_size, corner_zero_zone,
					                 corner_refinement_criteria);
				} else {
					cv::cornerSubPix(frame, corners, corner_window_size, corner_zero_zone, corner_refinement_criteria);
				}
			}
			cv::drawChessboardCorners(frame, chessboard_size, corners, patternfound);
		}

		std::string color_window_name, depth_window_name, ir_window_name, lh_camera_window_name;

		void ir_callback(const sensor_msgs::ImageConstPtr& msg) {
			cv::Mat frame = cv_bridge::toCvShare(msg, "mono16")->image;
			cv::normalize(frame, frame, 0, 65535, cv::NORM_MINMAX);
			if (draw_chessboard) {
				frame.convertTo(frame, CV_8U, 0.00390625);
				draw_chessboard_helper(frame);
			}
			cv::imshow(ir_window_name, frame);
		}

		void color_callback(const sensor_msgs::ImageConstPtr& msg) {
			cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
			if (draw_chessboard) {
				draw_chessboard_helper(frame);
			}
			cv::imshow(color_window_name, frame);
		}

		void depth_callback(const sensor_msgs::ImageConstPtr& msg) {
			cv::Mat frame = cv_bridge::toCvShare(msg, "mono16")->image;
			cv::normalize(frame, frame, 0, 65535, cv::NORM_MINMAX);
			cv::imshow(depth_window_name, frame);
		}

		void left_hand_camera_callback(const sensor_msgs::ImageConstPtr& msg) {
			std::lock_guard<std::mutex> lock(lh_cam_protection);
			last_lh_cam_frame = cv_bridge::toCvShare(msg, "bgr8")->image;
			cv::imshow(lh_camera_window_name, last_lh_cam_frame);

		}

	};

	calibration_capture_node* calibration_capture_node::instance = NULL;
	cv::TermCriteria calibration_capture_node::corner_refinement_criteria = cv::TermCriteria(
			CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
	cv::Size calibration_capture_node::corner_window_size = cv::Size(5, 5);
	cv::Size calibration_capture_node::corner_zero_zone = cv::Size(-1, -1);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "calibration_capture", ros::init_options::NoSigintHandler);
	rta_openni::calibration_capture_node node;
	signal(SIGINT, rta_openni::calibration_capture_node::handle_sigterm);

	return node.run();
}