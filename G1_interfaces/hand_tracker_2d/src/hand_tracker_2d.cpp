#include <iostream>
#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "kcftracker/kcftracker.hpp"
#include "hand_tracker_2d/hand_tracker_2d.h"

#include <dirent.h>

using namespace std;
using namespace cv;

HandTracker::HandTracker(string image_topic) :
nh_(),
is_tracking_(false),
is_drawing_wnd_(false),
init_tracker_(false),
HOG(true),
FIXEDWINDOW(false),
MULTISCALE(true),
SILENT(true),
LAB(false),
object_tag_(""),
tracker_(HOG, FIXEDWINDOW, MULTISCALE, LAB)
{
  snapshot_stamp_ = ros::Time(0);
  image_cache_.clear();

  const char * hand_topic = "/interact/hand_location";
  image_sub_ = nh_.subscribe(image_topic, 1, &HandTracker::image_callback, this);
  detector_sub_ = nh_.subscribe(hand_topic, 1, &HandTracker::hand_callback, this);

  image_transport::ImageTransport it(nh_);
  // display_pub_ = it.advertise("tracking_display", 1);
  bbox_pub_ = nh_.advertise<hand_tracker_2d::HandBBox>("/interact/tracking_bbox", 1);

  ROS_INFO("Ready to track hand, waiting for inital location from hand detector.");
}


void HandTracker::onMouse(int event, int x, int y)
{
  if (is_drawing_wnd_) {
    init_wnd_.x = min(x, init_corner_.x);
    init_wnd_.y = min(y, init_corner_.y);
    init_wnd_.width = abs(x - init_corner_.x);
    init_wnd_.height = abs(y - init_corner_.y);
    track_wnd_ = init_wnd_;
  }

  switch (event) {
    case EVENT_LBUTTONDOWN:
      is_drawing_wnd_ = true;
      snapshot_frame_ = frame_;
      object_tag_ = "";
      init_corner_ = Point(x, y);
      init_wnd_ = Rect(x, y, 0, 0);

      is_tracking_ = false;
      init_tracker_ = false;
      break;
    case EVENT_LBUTTONUP:
      is_drawing_wnd_ = false;
      if (init_wnd_.width > 0 && init_wnd_.height > 0) {
        is_tracking_ = true;
      }
      break;
  }
}

void HandTracker::hand_callback(const hand_tracker_2d::HandBBox msgs_hand)
{
  if (!frame_.empty()) {
    // snapshot_frame_ = frame_;
    sensor_msgs::Image msgs_image = getImageByStamp(msgs_hand.stamp);
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msgs_image, sensor_msgs::image_encodings::BGR8);
    snapshot_frame_ = cv_ptr->image;

    object_tag_ = "";

    init_tracker_ = false;
    is_tracking_ = true;
    init_wnd_.x = min(msgs_hand.box[0], msgs_hand.box[2]);
    init_wnd_.y = min(msgs_hand.box[1], msgs_hand.box[3]);
    init_wnd_.width = abs(msgs_hand.box[2] - msgs_hand.box[0]);
    init_wnd_.height = abs(msgs_hand.box[3] - msgs_hand.box[1]);
    track_wnd_ = init_wnd_;

    restartTrackingFromCache();
  }
}

void HandTracker::image_callback(const sensor_msgs::Image& msgs_image)
{
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msgs_image, sensor_msgs::image_encodings::BGR8);
    frame_ = cv_ptr->image;
    if (frame_.empty()) {
      return;
    }
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (is_tracking_) {
    // TODO: extend tracker to multiple objects
    trackSingleObject();

    // if (checkBBoxes(bbox_new_)) {
    //   // skip the new bounding box
    // } else {
    //   bbox_ = bbox_new_;
    // }
  }

  if (!is_drawing_wnd_) {
    frame_.copyTo(display_);
    drawBBoxes();
    imshow("Object Tracking", display_);
    waitKey(1);
  } else {
    snapshot_frame_.copyTo(display_);
    drawBBoxes();
    imshow("Object Tracking", display_);
    waitKey(1);
  }

  // sensor_msgs::ImagePtr display_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", display_).toImageMsg();
  // display_pub_.publish(display_msg);

  if (init_tracker_) {
    hand_tracker_2d::HandBBox bbox_msg;
    bbox_msg.stamp = msgs_image.header.stamp;
    bbox_msg.box.resize(4);
    bbox_msg.box[0] = track_wnd_.x;
    bbox_msg.box[1] = track_wnd_.y;
    bbox_msg.box[2] = track_wnd_.x + track_wnd_.width;
    bbox_msg.box[3] = track_wnd_.y + track_wnd_.height;
    bbox_pub_.publish(bbox_msg);
  }

  updateImageCache(msgs_image);
}

void HandTracker::trackSingleObject()
{
  if (!init_tracker_) {
    ROS_INFO("...tracking started (x,y,w,h)=(%d, %d, %d, %d)",
              init_wnd_.x, init_wnd_.y, init_wnd_.width, init_wnd_.height);
    tracker_.init(init_wnd_, snapshot_frame_);
    init_tracker_ = true;
    track_wnd_ = init_wnd_;
  } else {
    track_wnd_ = tracker_.update(frame_, confidence_);
    // cout << confidence_ << endl;
  }
}

int HandTracker::checkBBoxes() {
  return 0;
}

void HandTracker::drawBBoxes() {
  rectangle(display_, track_wnd_, Scalar(0,0,255), 3);
  if (!(object_tag_=="")) {
    putText(display_, object_tag_, Point(track_wnd_.x, track_wnd_.y-5), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,255,0), 2);
  }
}

void HandTracker::spin() {
  ros::Rate r(30);
  while (nh_.ok()) {
    ros::spinOnce();
    r.sleep();
  }
}

void HandTracker::updateImageCache(const sensor_msgs::Image& msgs_image) {
  image_cache_.push_back(msgs_image);
  oldest_stamp_ = image_cache_.front().header.stamp;
  while (oldest_stamp_ + ros::Duration(2) < msgs_image.header.stamp || image_cache_.size() > 50) {
    image_cache_.pop_front();
    oldest_stamp_ = image_cache_.front().header.stamp;
  }
  oldest_stamp_ = image_cache_.front().header.stamp;
  cout << "Image Cache size: " << image_cache_.size()
       << "  oldest stamp: " << oldest_stamp_
       << "  current stamp: " << msgs_image.header.stamp
       << endl;
}

sensor_msgs::Image& HandTracker::getImageByStamp(const ros::Time stamp) {
  // return the latest frame in debugging mode
  if (stamp == ros::Time(0)) {
    return image_cache_.back();
  }

  // find the frame with nearest timestamp
  while (image_cache_.size() > 1 && image_cache_.front().header.stamp < stamp) {
    image_cache_.pop_front();
  }
  
  return image_cache_.front();
}

void HandTracker::restartTrackingFromCache() {
  //consume the cached frames to catch up the tracking
  tracker_.init(init_wnd_, snapshot_frame_);
  init_tracker_ = true;
  track_wnd_ = init_wnd_;

  while (image_cache_.size() > 0) {
    sensor_msgs::Image image_tmp = image_cache_.front();
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_tmp, sensor_msgs::image_encodings::BGR8);
    frame_ = cv_ptr->image;
   
    track_wnd_ = tracker_.update(frame_, confidence_);
    image_cache_.pop_front();
  }
}


HandTracker *p_tracker;
void onMouse(int event, int x, int y, int, void*) {
  if (p_tracker) {
    p_tracker->onMouse(event, x, y);
  }
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "object_tracker_2d");

  namedWindow("Object Tracking", 0);
  setMouseCallback("Object Tracking", onMouse, 0);
  resizeWindow("Object Tracking", 640, 600);
  // moveWindow("Object Tracking", 0, 0);

  string input_image = "/camera/rgb/image";
  if (argc > 1) {
    input_image = argv[1];
  }

  ROS_INFO("Subscribe image topic: %s", input_image.c_str());

  HandTracker tracker(input_image);
  p_tracker = &tracker;

  tracker.spin();

  return 0;
}
