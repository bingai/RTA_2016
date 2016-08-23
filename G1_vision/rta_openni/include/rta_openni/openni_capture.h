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
#pragma once

//OpenCV
#include <opencv2/core/core.hpp>

//OpenNI
#include <openni2/OpenNI.h>

//stdilb
#include <array>
#include <deque>

namespace ros {
	class Time;
}

namespace rta_openni {

	/**
	 * OpenNI2 Xtion convenience wrapper class.
	 * This class is generally NOT thread-safe.
	 */
	class openni2_xtion_capture {

	public:
		static openni::VideoMode highres_color_mode, lowres_color_mode, depth_mode, lowres_ir_mode, highres_ir_mode;
		static const unsigned int hi_color_width, hi_color_height, lo_color_width, lo_color_height, depth_width,
				depth_height, lo_ir_width, lo_ir_height, hi_ir_width, hi_ir_height,
				hi_color_bytesize, lo_color_bytesize, depth_bytesize, hi_ir_bytesize, lo_ir_bytesize;

		openni2_xtion_capture(bool high_resolution_color = false, bool high_resolution_ir = false,
		                      const char* uri = openni::ANY_DEVICE);

		~openni2_xtion_capture();

		void unsafe_read(cv::Mat& color, cv::Mat& depth, ros::Time& time_stamp);

		void unsafe_read(cv::Mat& color, cv::Mat& depth, cv::Mat& depth_TS, ros::Time& time_stamp);

		void unsafe_read_ir(cv::Mat& ir, cv::Mat& depth, ros::Time& time_stamp);

		void unsafe_read_ir(cv::Mat& ir, cv::Mat& depth, cv::Mat& depth_TS, ros::Time& time_stamp);

		void safe_read(cv::Mat& color, cv::Mat& depth, ros::Time& time_stamp);

		void safe_read(cv::Mat& color, cv::Mat& depth, cv::Mat& depth_TS, ros::Time& time_stamp);

		void set_use_high_resolution_color(bool use_high_resolution_color);

		void set_use_high_resolution_ir(bool use_high_resolution_ir);

		float get_depth_horizontal_field_of_view() const;

		float get_color_horizontal_field_of_view() const;

		std::string get_string_id() const;

		bool emitter_is_on() const;

		void set_emitter_state(bool on);

		void toggle_emitter_state();

		void set_ir_stream_state(bool on);

		bool ir_stream_is_on();

	private:
		class init {
		public:
			init() ;
		};

		inline void check_color_frame_format(cv::Mat& color);

		inline void check_depth_frame_format(cv::Mat& depth);

		static init static_initializer;
		float depth_horizontal_field_of_view;
		float color_horizontal_field_of_view;


		//OpenNI stuff
		openni::VideoMode* current_color_mode;
		openni::VideoMode* current_ir_mode;
		unsigned int color_bytesize;
		unsigned int ir_bytesize;
		openni::VideoStream* streams[2];
		openni::VideoStream* streams_ir[2];
		openni::Device device;
		openni::VideoStream color_stream;
		openni::VideoStream depth_stream;
		openni::VideoStream ir_stream;
		bool emitter_on;
		bool ir_stream_on;

		//depth time-smoothing (TS)
		bool depth_smoothing_enabled;

		unsigned int depth_smoothing_queue_size;
		std::deque<cv::Mat> depth_frame_queue;
	};


}
