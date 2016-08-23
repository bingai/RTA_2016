#ifndef OBJECT_TRACKER_2D_H
#define OBJECT_TRACKER_2D_H

#include <algorithm>
#include <ros/ros.h>
#include <ros/time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "kcftracker/kcftracker.hpp"

#include "object_cls_msgs/GetObjectClassRequest.h"

using namespace std;
using namespace cv;

class ObjectTracker
{
public:

  ros::NodeHandle nh_;
  ros::Subscriber image_sub_;
  image_transport::Publisher display_pub_;
  ros::Publisher bbox_pub_;

  vector<Point2f> bbox_;
  vector<Point2f> bbox_new_;
  bool init_tracker_;
  bool is_tracking_;
  bool is_drawing_wnd_;

  ros::ServiceClient oc_client_;
  string object_tag_;

  Mat frame_;
  Mat gray_;
  Mat display_;
  Mat snapshot_frame_;
  Point init_corner_;
  Rect init_wnd_;
  Rect track_wnd_;
  float confidence_;

  bool HOG;
  bool FIXEDWINDOW;
  bool MULTISCALE;
  bool SILENT;
  bool LAB;

  // Create KCFTracker object
  KCFTracker tracker_;

  ObjectTracker(string image_topic);
  virtual ~ObjectTracker() {}

  void onMouse(int event, int x, int y);
  void image_callback(const sensor_msgs::Image& msgs_image);

  void trackSingleObject();
  int checkBBoxes();
  void drawBBoxes();

  void spin();
};

#endif  // OBJECT_TRACKER_2D_H
