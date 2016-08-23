//  ================================================================
//  Created by Gregory Kramida on 6/30/16.
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
#include <rta_openni/openni_capture.h>

//OpenNI2
#include <openni2/PS1080.h>

//TODO: remove ROS dependency from this part of the code
//ROS
#include <ros/ros.h>

//opencv
#include <opencv2/imgproc/imgproc.hpp>


//system/boost
#include <iostream>
#include <boost/algorithm/string/replace.hpp>


#define SAMPLE_READ_WAIT_TIMEOUT 2000 //ms

//exception handling mode
#if !defined(RTA_OPENNI_ALLOW_EXCEPTIONS) || RTA_OPENNI_ALLOW_EXCEPTIONS == 0
#define NO_EXCEPTIONS
#endif

#ifndef NO_EXCEPTIONS
#include <stdexcept>
#endif

namespace rta_openni {

#ifdef NO_EXCEPTIONS
#define err(message) std::cout << message; std::abort()
#else
#define err(message) throw std::runtime_error(message);
#endif

	inline static
	void check_openni_error(openni::Status status) {
		if (status != openni::STATUS_OK) {
			std::stringstream ss;
			ss << "OpenNI failed with status " << openni::STATUS_OK << ", extended error: " <<
			   openni::OpenNI::getExtendedError() << std::endl;
			err(ss.str());
		}
	}

	/** \brief Smooth a depth image using a moving average filter.
     *  \param[in]  depth_images a vector containing depth images
     *  \param[out] depth_TS  a smoothed depth image
     */
	inline static
	void temproally_smooth_depth_image(const std::deque<cv::Mat>& depth_images, cv::Mat& depth_TS) {

		if (depth_images.size() < 1) {
			std::cout << "[utl::kinect::smoothDepthImage] input depth images are empty." << std::endl;
			std::abort();
		}

		// Prepare variables
		cv::Mat smoothed_depth = cv::Mat::zeros(depth_images[0].size(), CV_32F);
		cv::Mat pixel_weights = cv::Mat::zeros(depth_images[0].size(), CV_32F);

		//TODO: this can be made much faster if the cumulative image is preserved for next time
		// (in case queueize doesn't change, simply subtracting the oldest image & adding the new one is good enough)
		// Update cumulative image
		for (size_t imId = 0; imId < depth_images.size(); imId++) {
			// Convert to float
			cv::Mat raw_depth;
			depth_images[imId].convertTo(raw_depth, CV_32F);

			// Get valid pixel mask
			cv::Mat validDepth;
			cv::threshold(raw_depth, validDepth, 0, 1.0, CV_THRESH_BINARY);

			// Update weights
			cv::add(pixel_weights, validDepth, pixel_weights);

			// Add valid pixels only
			validDepth.convertTo(validDepth, CV_8U);
			cv::add(smoothed_depth, raw_depth, smoothed_depth, validDepth);
		}

		// Normalize
		float minNumValidPixels = static_cast<float>(depth_images.size()) / 2.0f;
		for (size_t x = 0; x < static_cast<size_t>(smoothed_depth.cols); x++) {
			for (size_t y = 0; y < static_cast<size_t>(smoothed_depth.rows); y++) {
				float curPixelWeight = pixel_weights.at<float>(y, x);
				if (curPixelWeight < minNumValidPixels)
					smoothed_depth.at<float>(y, x) = 0.0f;
				else
					smoothed_depth.at<float>(y, x) = smoothed_depth.at<float>(y, x) / curPixelWeight;
			}
		}
		// Convert back to CV_16U
		smoothed_depth.convertTo(depth_TS, CV_16U);
	}

	static std::string opencv_matrix_type_str(int number) {
		// find type
		int imgTypeInt = number % 8;
		std::string imgTypeString;

		switch (imgTypeInt) {
			case 0:
				imgTypeString = "8U";
				break;
			case 1:
				imgTypeString = "8S";
				break;
			case 2:
				imgTypeString = "16U";
				break;
			case 3:
				imgTypeString = "16S";
				break;
			case 4:
				imgTypeString = "32S";
				break;
			case 5:
				imgTypeString = "32F";
				break;
			case 6:
				imgTypeString = "64F";
				break;
			default:
				break;
		}

		// find channel
		int channel = (number / 8) + 1;

		std::stringstream type;
		type << "CV_" << imgTypeString << "C" << channel;

		return type.str();
	}

	// set up modes
	const unsigned int
			openni2_xtion_capture::hi_color_width = 1280,
			openni2_xtion_capture::hi_color_height = 1024,
			openni2_xtion_capture::lo_color_width = 640,
			openni2_xtion_capture::lo_color_height = 480,
			openni2_xtion_capture::depth_width = 640,
			openni2_xtion_capture::depth_height = 480,
			openni2_xtion_capture::hi_ir_width = 1280,
			openni2_xtion_capture::hi_ir_height = 1024,
			openni2_xtion_capture::lo_ir_width = 640,
			openni2_xtion_capture::lo_ir_height = 480;

	//3 channels
	const unsigned int openni2_xtion_capture::hi_color_bytesize =
			hi_color_height * hi_color_width * 3 * sizeof(unsigned char);
	const unsigned int openni2_xtion_capture::lo_color_bytesize =
			lo_color_height * lo_color_width * 3 * sizeof(unsigned char);
	const unsigned int openni2_xtion_capture::depth_bytesize = depth_height * depth_width * sizeof(unsigned short);
	const unsigned int openni2_xtion_capture::hi_ir_bytesize = hi_ir_height * hi_ir_width * sizeof(unsigned short);
	const unsigned int openni2_xtion_capture::lo_ir_bytesize = lo_ir_height * lo_ir_width * sizeof(unsigned short);

	openni::VideoMode openni2_xtion_capture::highres_color_mode;
	openni::VideoMode openni2_xtion_capture::lowres_color_mode;
	openni::VideoMode openni2_xtion_capture::depth_mode;
	openni::VideoMode openni2_xtion_capture::highres_ir_mode;
	openni::VideoMode openni2_xtion_capture::lowres_ir_mode;

	//static initialization block
	openni2_xtion_capture::init::init() {
		openni::OpenNI::initialize();
		const int fps = 30;
		highres_color_mode.setResolution(hi_color_width, hi_color_height);
		highres_color_mode.setFps(fps);
		highres_color_mode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);

		lowres_color_mode.setResolution(lo_color_width, lo_color_height);
		lowres_color_mode.setFps(fps);
		lowres_color_mode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);

		depth_mode.setResolution(depth_width, depth_height);
		depth_mode.setFps(fps);
		depth_mode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);

		lowres_ir_mode.setResolution(lo_ir_width, lo_ir_height);
		lowres_ir_mode.setFps(fps);
		lowres_ir_mode.setPixelFormat(openni::PIXEL_FORMAT_GRAY16);

		highres_ir_mode.setResolution(hi_ir_width, hi_ir_height);
		highres_ir_mode.setFps(fps);
		highres_ir_mode.setPixelFormat(openni::PIXEL_FORMAT_GRAY16);
	}

	openni2_xtion_capture::init openni2_xtion_capture::static_initializer;//static initialization of color modes

	openni2_xtion_capture::openni2_xtion_capture(bool high_resolution_color, bool high_resolution_ir,
	                                             const char* uri) :
			current_color_mode(high_resolution_color ? &highres_color_mode : &lowres_color_mode),
			current_ir_mode(high_resolution_ir ? &highres_ir_mode : &lowres_ir_mode),
			color_bytesize(high_resolution_color ? hi_color_bytesize : lo_color_bytesize),
			ir_bytesize(high_resolution_ir ? hi_ir_bytesize : lo_ir_bytesize),
			emitter_on(true),
			ir_stream_on(false),
			streams{&color_stream, &depth_stream},
			streams_ir{&ir_stream, &depth_stream},
			depth_smoothing_queue_size(15), depth_smoothing_enabled(true),
			depth_frame_queue() {
		// set up device & streams
		check_openni_error(device.open(uri));
		device.setDepthColorSyncEnabled(true);
		device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);

		check_openni_error(color_stream.create(device, openni::SENSOR_COLOR));
		check_openni_error(color_stream.setVideoMode(*current_color_mode));
		check_openni_error(color_stream.setMirroringEnabled(false));

		check_openni_error(depth_stream.create(device, openni::SENSOR_DEPTH));
		check_openni_error(depth_stream.setVideoMode(depth_mode));
		check_openni_error(depth_stream.setMirroringEnabled(false));

		check_openni_error(ir_stream.create(device, openni::SENSOR_IR));
		check_openni_error(ir_stream.setVideoMode(*current_ir_mode));
		check_openni_error(ir_stream.setMirroringEnabled(false));

		// start streams
		check_openni_error(color_stream.start());
		check_openni_error(depth_stream.start());

		depth_horizontal_field_of_view = depth_stream.getHorizontalFieldOfView();
		color_horizontal_field_of_view = color_stream.getHorizontalFieldOfView();
	}

	openni2_xtion_capture::~openni2_xtion_capture() {
		//TODO: this should be done in a static function on SIGINT
		depth_stream.stop();
		if (ir_stream_on) {
			ir_stream.stop();
		} else {
			color_stream.stop();
		}
		device.close();
		openni::OpenNI::shutdown();//TODO: should occur before termination of program, not here
	}

	void openni2_xtion_capture::check_color_frame_format(cv::Mat& color) {
		if (color.empty()) {
			color = cv::Mat(cv::Size(current_color_mode->getResolutionX(), current_color_mode->getResolutionY()),
			                CV_8UC3);
		} else {
			if (color.cols != current_color_mode->getResolutionX() || color.rows != current_color_mode->getResolutionY()
			    || color.type() != CV_8UC3) {
				std::stringstream ss;
				ss << "Wrong color frame resolution or type. Expected " << current_color_mode->getResolutionX()
				   << "x" << current_color_mode->getResolutionY() << ", type CV_8UC3 (" << CV_8UC3 << "); got "
				   << color.cols << "x" << color.rows << ", type " << opencv_matrix_type_str(color.type()) << " (" <<
				   color.type() << ")." << std::endl;
				err(ss.str());
			}
		}
	}

	void openni2_xtion_capture::check_depth_frame_format(cv::Mat& depth) {
		if (depth.empty()) {
			depth = cv::Mat(cv::Size(depth_mode.getResolutionX(), depth_mode.getResolutionY()), CV_16UC1);
		} else {
			if (depth.cols != depth_mode.getResolutionX() || depth.rows != depth_mode.getResolutionY()
			    || depth.type() != CV_16UC1) {
				std::stringstream ss;
				ss << "Wrong depth frame resolution or type. Expected " << depth_mode.getResolutionX()
				   << "x" << depth_mode.getResolutionY() << ", type CV_8UC3 (" << CV_16UC1 << "); got "
				   << depth.cols << "x" << depth.rows << ", type " << opencv_matrix_type_str(depth.type()) << " (" <<
				   depth.type() << ")." << std::endl;
				err(ss.str());
			}
		}
	}

	void openni2_xtion_capture::safe_read(cv::Mat& color, cv::Mat& depth, ros::Time& time_stamp) {
		check_color_frame_format(color);
		check_depth_frame_format(depth);
		unsafe_read(color, depth, time_stamp);
	}

	void openni2_xtion_capture::safe_read(cv::Mat& color, cv::Mat& depth, cv::Mat& depth_TS, ros::Time& time_stamp) {
		check_color_frame_format(color);
		check_depth_frame_format(depth);
		check_depth_frame_format(depth_TS);
		unsafe_read(color, depth, depth_TS, time_stamp);
	}

	static inline void
	frame_read_helper(openni::VideoFrameRef& openni_frame, openni::VideoStream& stream, unsigned int bytesize,
	                  cv::Mat& mat) {
		check_openni_error(stream.readFrame(&openni_frame));
		if (openni_frame.isValid()) {
			const openni::RGB888Pixel* colorImagePix = (const openni::RGB888Pixel*) openni_frame.getData();
			memcpy(mat.data, colorImagePix, bytesize);
		}
	}

	void openni2_xtion_capture::unsafe_read(cv::Mat& color, cv::Mat& depth, ros::Time& time_stamp) {
		int ready_stream = -1;
		openni::VideoFrameRef openni_frame;
		time_stamp = ros::Time::now();
#define SIMPLE_READ
#ifdef SIMPLE_READ
		check_openni_error(openni::OpenNI::waitForAnyStream(streams, 2, &ready_stream, SAMPLE_READ_WAIT_TIMEOUT));
		frame_read_helper(openni_frame, color_stream, color_bytesize, color);
		cv::cvtColor(color, color, cv::COLOR_RGB2BGR);
		frame_read_helper(openni_frame, depth_stream, depth_bytesize, depth);
#else
		for (int i_stream = 0; i_stream < 2; i_stream++){
			check_openni_error(openni::OpenNI::waitForAnyStream(streams, 2, &ready_stream, SAMPLE_READ_WAIT_TIMEOUT));
			switch (ready_stream) {
				case 0:
					frame_read_helper(openni_frame,color_stream,color_bytesize,color);
					cv::cvtColor(color, color, cv::COLOR_RGB2BGR);
					break;
				case 1:
					frame_read_helper(openni_frame,depth_stream,depth_bytesize,depth);
					break;
				default:
					break;
			}
		}
#endif
	}

	/**
	 * Attempt to read in the next frames (ir, depth, depth_TS) without checking the input matix type or dimensions.
	 * @param[out] ir - matrix where to store the IR image
	 * @param[out] color - matrix where to store the color image
	 * @param[out] depth - matrix where to store the depth image
	 * @param[out] time_stamp - current time stamp
	 */
	void openni2_xtion_capture::unsafe_read_ir(cv::Mat& ir, cv::Mat& depth, ros::Time& time_stamp) {
		int ready_stream = -1;
		openni::VideoFrameRef openni_frame;
		time_stamp = ros::Time::now();
#define SIMPLE_READ
#ifdef SIMPLE_READ
		check_openni_error(openni::OpenNI::waitForAnyStream(streams_ir, 2, &ready_stream, SAMPLE_READ_WAIT_TIMEOUT));
		frame_read_helper(openni_frame, depth_stream, depth_bytesize, depth);
		frame_read_helper(openni_frame, ir_stream, ir_bytesize, ir);
#else
		for (int i_stream = 0; i_stream < 2; i_stream++){
			check_openni_error(openni::OpenNI::waitForAnyStream(streams, 2, &ready_stream, SAMPLE_READ_WAIT_TIMEOUT));
			switch (ready_stream) {
				case 0:
					frame_read_helper(openni_frame,color_stream,color_bytesize,color);
					cv::cvtColor(color, color, cv::COLOR_RGB2BGR);
					break;
				case 1:
					frame_read_helper(openni_frame,depth_stream,depth_bytesize,depth);
					break;
				case 2:
					frame_read_helper(openni_frame,ir_stream,ir_bytesize,ir);
					break;
				default:
					break;
			}
		}
#endif
	}

	/**
	 * Attempt to read in the next frames (color, depth, depth_TS) without checking the input matix type or dimensions.
	 */
	void openni2_xtion_capture::unsafe_read(cv::Mat& color, cv::Mat& depth, cv::Mat& depth_TS, ros::Time& time_stamp) {
		this->unsafe_read(color, depth, time_stamp);
		if (depth_smoothing_enabled) {
			while (depth_frame_queue.size() >= static_cast<size_t>(depth_smoothing_queue_size))
				depth_frame_queue.pop_front();
			depth_frame_queue.push_back(depth.clone());
			temproally_smooth_depth_image(depth_frame_queue, depth_TS);
		}
	}

	/**
	 * Attempt to read in the next frames (color, depth, depth_TS) without checking the input matix type or dimensions.
	 */
	void openni2_xtion_capture::unsafe_read_ir(cv::Mat& ir, cv::Mat& depth, cv::Mat& depth_TS,
	                                           ros::Time& time_stamp) {
		this->unsafe_read_ir(ir, depth, time_stamp);
		if (depth_smoothing_enabled) {
			while (depth_frame_queue.size() >= static_cast<size_t>(depth_smoothing_queue_size))
				depth_frame_queue.pop_front();
			depth_frame_queue.push_back(depth.clone());
			temproally_smooth_depth_image(depth_frame_queue, depth_TS);
		}
	}


	void openni2_xtion_capture::set_use_high_resolution_color(bool use_high_resolution_color) {
		if (use_high_resolution_color && this->current_color_mode == &lowres_color_mode) {
			color_stream.stop();
			current_color_mode = &highres_color_mode;
			color_bytesize = hi_color_bytesize;
			color_stream.setVideoMode(*current_color_mode);
			check_openni_error(color_stream.start());
		} else if (!use_high_resolution_color && this->current_color_mode == &highres_color_mode) {
			color_stream.stop();
			current_color_mode = &lowres_color_mode;
			color_bytesize = lo_color_bytesize;
			color_stream.setVideoMode(*current_color_mode);
			check_openni_error(color_stream.start());
		}
	}

	void openni2_xtion_capture::set_use_high_resolution_ir(bool use_high_resolution_ir) {
		if (use_high_resolution_ir && this->current_ir_mode == &lowres_ir_mode) {
			ir_stream.stop();
			current_ir_mode = &highres_ir_mode;
			ir_bytesize = hi_ir_bytesize;
			ir_stream.setVideoMode(*current_ir_mode);
			check_openni_error(ir_stream.start());
		} else if (!use_high_resolution_ir && this->current_ir_mode == &highres_ir_mode) {
			ir_stream.stop();
			current_ir_mode = &lowres_ir_mode;
			ir_bytesize = lo_ir_bytesize;
			ir_stream.setVideoMode(*current_ir_mode);
			check_openni_error(ir_stream.start());
		}
	}

	float openni2_xtion_capture::get_depth_horizontal_field_of_view() const {
		return this->depth_horizontal_field_of_view;
	}

	float openni2_xtion_capture::get_color_horizontal_field_of_view() const {
		return this->color_horizontal_field_of_view;
	}

	std::string openni2_xtion_capture::get_string_id() const{
		const openni::DeviceInfo device_info = this->device.getDeviceInfo();
		std::string ID_str = std::string(device_info.getName()) + "_" + std::string(device_info.getVendor());

		boost::replace_all(ID_str, "/", "");
		boost::replace_all(ID_str, ".", "");
		boost::replace_all(ID_str, "@", "");

		return ID_str;
	}

	bool openni2_xtion_capture::emitter_is_on() const {
		/*
		 * TODO: proper getter doesn't work on XtionPro, use workaround
		 *bool status;
		 *	check_openni_error(device.getProperty(XN_MODULE_PROPERTY_EMITTER_STATE, &status));
		 *
		 *return status;
		 */
		return emitter_on;
	}

	void openni2_xtion_capture::set_emitter_state(bool on) {
		if(on == emitter_on){
			return;
		}
		check_openni_error(device.setProperty(XN_MODULE_PROPERTY_EMITTER_STATE, on));
		emitter_on = on;
	}

	void openni2_xtion_capture::toggle_emitter_state() {
		this->set_emitter_state(!this->emitter_is_on());
	}

	void openni2_xtion_capture::set_ir_stream_state(bool on) {
		if (ir_stream_on == on) {
			return;
		} else {
			if (on) {
				color_stream.stop();
				check_openni_error(ir_stream.start());
				emitter_on = true;
			} else {
				ir_stream.stop();
				check_openni_error(color_stream.start());
			}
			ir_stream_on = on;
		}
	}

	bool openni2_xtion_capture::ir_stream_is_on() {
		return ir_stream_on;
	}

}
