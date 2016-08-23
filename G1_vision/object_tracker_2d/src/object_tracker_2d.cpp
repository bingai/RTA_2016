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
#include "object_tracker_2d/object_tracker_2d.h"

#include "object_cls_msgs/BBox.h"
#include "object_cls_msgs/GetObjectClass.h"
#include "object_cls_msgs/GetObjectClassRequest.h"

#include <dirent.h>

using namespace std;
using namespace cv;

ObjectTracker::ObjectTracker(string image_topic) :
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
  image_sub_ = nh_.subscribe(image_topic, 1, &ObjectTracker::image_callback, this);

  image_transport::ImageTransport it(nh_);
  display_pub_ = it.advertise("tracking_display", 1);
  bbox_pub_ = nh_.advertise<object_cls_msgs::BBox>("tracking_bbox", 1);

  oc_client_ = nh_.serviceClient<object_cls_msgs::GetObjectClass>("get_object_class");

  ROS_INFO("Ready to track objects, please draw a rectangle in the image window to start tracking.");
}


void ObjectTracker::onMouse(int event, int x, int y)
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

      object_cls_msgs::GetObjectClass srv;
      srv.request.images.resize(1);
      sensor_msgs::ImagePtr request_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", snapshot_frame_).toImageMsg();
      srv.request.images[0] = *request_image;
      srv.request.bbox_lists.resize(1);
      object_cls_msgs::BBoxList bbox_list;
      bbox_list.bbox_list.resize(1);
      object_cls_msgs::BBox bbox;
      bbox.id = "obj_0";
      bbox.box.resize(4);
      bbox.box[0] = init_wnd_.x;
      bbox.box[1] = init_wnd_.y;
      bbox.box[2] = init_wnd_.x + init_wnd_.width;
      bbox.box[3] = init_wnd_.y + init_wnd_.height;
      bbox_list.bbox_list[0] = bbox;
      srv.request.bbox_lists[0] = bbox_list;
      srv.request.task = object_cls_msgs::GetObjectClassRequest::TASK_TYPE_DISH;

      if (oc_client_.call(srv)) {

        // switch (srv.response.predictions[0]) {
        //   case object_cls_msgs::GetObjectClassRequest::OBJ_DISH_BOWL:
        //     object_tag_ = string("bowl");
        //     break;
        //   case object_cls_msgs::GetObjectClassRequest::OBJ_DISH_CUP:
        //     object_tag_ = string("cup");
        //     break;
        //   case object_cls_msgs::GetObjectClassRequest::OBJ_DISH_MUG:
        //     object_tag_ = string("mug");
        //     break;
        //   case object_cls_msgs::GetObjectClassRequest::OBJ_DISH_PLATE:
        //     object_tag_ = string("plate");
        // }

        object_tag_ = string(srv.response.predictions[0]);
        ROS_INFO("Object class is: %s", object_tag_.c_str());

      } else {
        ROS_ERROR("Failed to call service GetObjectClass");
      }

      break;
  }

}

void ObjectTracker::image_callback(const sensor_msgs::Image& msgs_image)
{

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msgs_image, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  frame_ = cv_ptr->image;
  if (frame_.empty()) {
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

  sensor_msgs::ImagePtr display_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", display_).toImageMsg();
  display_pub_.publish(display_msg);

  if (is_tracking_) {
    object_cls_msgs::BBox bbox_msg;
    bbox_msg.id = "tracking_obj_0";
    bbox_msg.box.resize(4);
    bbox_msg.box[0] = track_wnd_.x;
    bbox_msg.box[1] = track_wnd_.y;
    bbox_msg.box[2] = track_wnd_.x + track_wnd_.width;
    bbox_msg.box[3] = track_wnd_.y + track_wnd_.height;
    bbox_pub_.publish(bbox_msg);
  }
}

void ObjectTracker::trackSingleObject()
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

int ObjectTracker::checkBBoxes() {
  return 0;
}

void ObjectTracker::drawBBoxes() {
  rectangle(display_, track_wnd_, Scalar(0,0,255), 3);
  if (!(object_tag_=="")) {
    putText(display_, object_tag_, Point(track_wnd_.x, track_wnd_.y-5), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,255,0), 2);
  }
}

void ObjectTracker::spin() {
  ros::Rate r(30);
  while (nh_.ok()) {
    ros::spinOnce();
    r.sleep();
  }
}

ObjectTracker *p_tracker;
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

  ObjectTracker tracker(input_image);
  p_tracker = &tracker;

  tracker.spin();

  return 0;
}
