#ifndef OBJECT_TRACKER_2D_H
#define OBJECT_TRACKER_2D_H

#include <algorithm>
#include <deque>
#include <ros/ros.h>
#include <ros/time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>

#include "kcftracker/kcftracker.hpp"
#include "hand_tracker_2d/HandBBox.h"

using namespace std;
using namespace cv;

class HandTracker
{
public:

  ros::NodeHandle nh_;
  ros::Subscriber image_sub_;
  ros::Subscriber detector_sub_;
  // image_transport::Publisher display_pub_;
  ros::Publisher bbox_pub_;

  vector<Point2f> bbox_;
  vector<Point2f> bbox_new_;
  bool init_tracker_;
  bool is_tracking_;
  bool is_drawing_wnd_;

  string object_tag_;

  Mat frame_;
  Mat gray_;
  Mat display_;
  Mat snapshot_frame_;
  Point init_corner_;
  Rect init_wnd_;
  Rect track_wnd_;
  float confidence_;

  deque<sensor_msgs::Image> image_cache_;
  ros::Time oldest_stamp_;
  ros::Time snapshot_stamp_;

  bool HOG;
  bool FIXEDWINDOW;
  bool MULTISCALE;
  bool SILENT;
  bool LAB;

  // Create KCFTracker object
  KCFTracker tracker_;

  HandTracker(string image_topic);
  virtual ~HandTracker() {}

  void onMouse(int event, int x, int y);
  void hand_callback(const hand_tracker_2d::HandBBox msgs_hand);
  void image_callback(const sensor_msgs::Image& msgs_image);

  void trackSingleObject();
  int checkBBoxes();
  void drawBBoxes();

  void updateImageCache(const sensor_msgs::Image& msgs_image);
  sensor_msgs::Image& getImageByStamp(const ros::Time stamp);
  void restartTrackingFromCache();

  void spin();
};

#endif  // OBJECT_TRACKER_2D_H
