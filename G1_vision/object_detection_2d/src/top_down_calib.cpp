#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

// #include <sisyphus/registration_utilities.hpp>

using namespace cv;
using namespace std;

class BoardTracker
{

public:

  ros::NodeHandle nh_;
  string camera_info_tp_;
  string image_tp_;
  ros::Subscriber camera_sub_;
  ros::Subscriber image_sub_;
  image_transport::Publisher display_pub_;
  ros::Publisher marker_array_pub_;
  tf::TransformBroadcaster tf_br_;
  tf::Transform tform_;

  string parent_tf_;
  string tf_;

  // image setting
  int width_;
  int height_;

  // board setting
  int n_rows_;
  int n_cols_;
  double square_size_;
  Size patternsize_;
  vector<Point2f> corners_;
  vector<Point3f> corners_3d_;
  vector<Point3f> field_corner_3d_;

  Mat cameraMatrix_;
  Mat cameraInv_;
  Mat distCoeffs_;
  Eigen::Matrix3f c_tmp_;
  Eigen::Matrix3f ci_tmp_;
  Eigen::Matrix3f R_;
  Eigen::Vector3f t_;

  Point3f table_scale;
  Point3f table_position;
  double center_x_;
  double center_y_;
  double fix_z_;

  Mat frame_;
  Mat display_;

  bool has_camera_info_;
  bool has_extrinsic_calib_;

  BoardTracker() :
    nh_("~"),
    cameraMatrix_(Mat(3,3,DataType<double>::type)),
    distCoeffs_(Mat(4,1,DataType<double>::type)),
    parent_tf_("/base"),
    tf_("/top_down_camera_link"),
    width_(1280),
    height_(1024),
    has_camera_info_(false),
    has_extrinsic_calib_(false)
  {
    nh_.param("camera", camera_info_tp_, string("/camera/camera_info"));
    nh_.param("image", image_tp_, string("/camera/image_color"));
    camera_sub_ = nh_.subscribe(camera_info_tp_, 1, &BoardTracker::camera_callback, this);
    image_sub_ = nh_.subscribe(image_tp_, 1, &BoardTracker::image_callback, this);

    ROS_INFO("Using camera_info: %s, image: %s", camera_info_tp_.c_str(), image_tp_.c_str());

    marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("markers_3d", 1);

    image_transport::ImageTransport it(nh_);
    display_pub_ = it.advertise("corner_display", 1);

    // init the default board
    n_rows_ = 6;
    n_cols_ = 9;
    square_size_ = 0.027;  // 27mm
    patternsize_ = Size(n_cols_, n_rows_);

    // init table position
    table_position.x = 0.72;
    table_position.y = 0.00;
    table_position.z = -0.69;

    table_scale.x = 0.8;
    table_scale.y = 1.6;
    table_scale.z = 1.0;

    nh_.param("center_x", center_x_, double(table_position.x));
    nh_.param("center_y", center_y_, double(table_position.y));
    nh_.param("center_z", fix_z_, double(table_position.z + table_scale.z * 0.5));

    ROS_INFO("Boarder center: (%.04f, %.04f)  height: %.04f", center_x_, center_y_, fix_z_);

    init_3d_pattern();

    distCoeffs_.at<double>(0) = 0;
    distCoeffs_.at<double>(1) = 0;
    distCoeffs_.at<double>(2) = 0;
    distCoeffs_.at<double>(3) = 0;


    ROS_INFO("Starting board tracker");

  }

  virtual ~BoardTracker() {}

  void init_3d_pattern() {
    for (int i = 0; i < n_rows_; i ++ ) {
      for (int j = 0; j < n_cols_; j ++ ) {
        int index = i * n_cols_ + j;
        Point3f pt();
        // x+: forward;  y+: left;  z+: up
        corners_3d_.push_back(Point3f(square_size_ * (n_rows_/2.0-0.5-i) + center_x_,
                                      square_size_ * (n_cols_/2.0-0.5-j) + center_y_,
                                      fix_z_));
      }
    }
  }

  void publish_3d_markers() {
    visualization_msgs::MarkerArray markers_3d;

    for (int i = 0; i < corners_3d_.size(); i++) {
      markers_3d.markers.push_back(get_arrow_marker(corners_3d_[i], i+100));
    }

    for (int i = 0; i < field_corner_3d_.size(); i++) {
      markers_3d.markers.push_back(get_arrow_marker(field_corner_3d_[i], i+100+corners_3d_.size(), 5.0));
    }

    marker_array_pub_.publish(markers_3d);
  }


  visualization_msgs::Marker get_arrow_marker(Point3f position, int id, double scale = 1.0) {

    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "/base";
    arrow.header.stamp = ros::Time::now();
    arrow.ns = "table";
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.id = id;
    arrow.lifetime = ros::Duration();

    arrow.color.r = 1.0f;
    arrow.color.g = 0.2f + (id-100)/(n_rows_*n_cols_)*0.8f;
    arrow.color.b = 0.0f;
    arrow.color.a = 1.0f;

    arrow.pose.position.x = position.x;
    arrow.pose.position.y = position.y;
    arrow.pose.position.z = position.z;
    arrow.pose.orientation.x = 0.0;
    arrow.pose.orientation.y = -1.0;
    arrow.pose.orientation.z = 0.0;
    arrow.pose.orientation.w = 1.0;

    arrow.scale.x = 0.010 * scale;
    arrow.scale.y = 0.005 * scale;
    arrow.scale.z = 0.005 * scale;

    return arrow;
  }


  void camera_callback(const sensor_msgs::CameraInfo& msgs_camera) {
    sensor_msgs::CameraInfo camera_info = msgs_camera;
    has_camera_info_ = true;
    for (int i = 0; i < 9; i ++) {
      cameraMatrix_.at<double>(i) = camera_info.K[i];
    }

    width_ = camera_info.width;
    height_ = camera_info.height;

    cameraInv_ = cameraMatrix_.inv();
    cv2eigen(cameraInv_, ci_tmp_);
    cv2eigen(cameraMatrix_, c_tmp_);

    ROS_INFO("received camera info: %f %f %f %f %f %f %f %f %f", camera_info.K[0], camera_info.K[1], camera_info.K[2],
                camera_info.K[3], camera_info.K[4], camera_info.K[5],
                camera_info.K[6], camera_info.K[7], camera_info.K[8]);
    ROS_INFO("   image size: %d x %d", camera_info.height, camera_info.width);

    camera_sub_.shutdown();
  }

  void image_callback(const sensor_msgs::Image& msgs_image) {

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

    field_corner_3d_.clear();

    // frame_.copyTo(display_);
    if (find_and_draw_corners(frame_, display_)) {
      // estimation pose
      find_pose();
    }

    sensor_msgs::ImagePtr display_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", display_).toImageMsg();
    display_pub_.publish(display_msg);
  }

  bool find_and_draw_corners(Mat input, Mat & display) {
    Mat gray;
    cvtColor(input, gray, CV_RGB2GRAY);
    input.copyTo(display);

    bool patternfound = findChessboardCorners(gray, patternsize_, corners_, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

    if (patternfound) {
      // refine the detected corner locations
      double min_distance = 1e10;
      for (int i = 0; i < n_rows_; i ++) {
        for (int j = 0; j < n_cols_-1; j ++) {
          int index = i*n_cols_ + j;
          min_distance = min(min_distance, norm(corners_[index]-corners_[index+1]));
        }
      }

      for (int i = 0; i < n_rows_-1; i ++) {
        for (int j = 0; j < n_cols_; j ++) {
          int index = i*n_cols_ + j;
          min_distance = min(min_distance, norm(corners_[index]-corners_[index+n_cols_]));
        }
      }
      int radius = int(ceil(min_distance * 0.5));
      cornerSubPix(gray, corners_, Size(radius, radius), Size(-1, -1),
            TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

      drawChessboardCorners(display, patternsize_, corners_, patternfound);

      return true;
    }

    return false;
  }

  void find_pose() {
    if (!has_camera_info_) {
      return;
    }

    // print the first row
    for (int i = 0; i < 9; i ++ ) {
      cout << i << "  3d: " << corners_3d_[i] << "  2d: " << corners_[i] << endl;
    }
    Mat rvec, rmat, tvec;
    bool ret = solvePnP(corners_3d_, corners_, cameraMatrix_,  distCoeffs_, rvec, tvec);

    Rodrigues(rvec, rmat);
    Eigen::Matrix3f R_tmp;
    Eigen::Vector3f t_tmp;
    cv2eigen(rmat, R_tmp);
    cv2eigen(tvec, t_tmp);

    R_ = R_tmp.transpose();
    t_ = -R_*t_tmp;

    eigen2cv(R_, rmat);
    eigen2cv(t_, tvec);

    cout << "cameraMatrix: " << cameraMatrix_ << endl;
    cout << "distCoeffs: " << distCoeffs_ << endl;
    cout << "R: " << rmat << endl;
    cout << "t: " << tvec << endl;

    tform_.setOrigin( tf::Vector3(t_(0), t_(1), t_(2)) );
    tf::Quaternion q;
    tf::Matrix3x3 R_tf;
    R_tf.setValue(R_(0,0), R_(0,1), R_(0,2), R_(1,0), R_(1,1), R_(1,2), R_(2,0), R_(2,1), R_(2,2));
    R_tf.getRotation(q);
    tform_.setRotation(q);

    cout << "quaternion: [" << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ",]" << endl;

    vector<Point2f> field_corner_2d;
    field_corner_2d.push_back(Point2f( 0, 0 ));
    field_corner_2d.push_back(Point2f( height_, 0 ));
    field_corner_2d.push_back(Point2f( height_, width_ ));
    field_corner_2d.push_back(Point2f( 0, width_ ));

    // double dz = R_tmp(2,0) * corners_3d_[0].x + R_tmp(2,1) * corners_3d_[0].y + R_tmp(2,2)*corners_3d_[0].z + t_tmp(2);

    field_corner_3d_.clear();
    for (int i = 0; i < field_corner_2d.size(); i ++ ) {

      double u = field_corner_2d[i].x;
      double v = field_corner_2d[i].y;
      Eigen::Vector3f h_pt(u, v, 1);
      Eigen::Vector3f pt1 = R_*ci_tmp_*h_pt;

      double s = pt1(2) / (fix_z_ - t_(2));

      Eigen::Vector3f pt3d = pt1/s + t_;
      field_corner_3d_.push_back(Point3f(pt3d(0), pt3d(1), pt3d(2)));
    }

    for (int i = 0; i < field_corner_3d_.size() && i < 4; i ++ ) {
      cout << "   proj_corners_3d: " << field_corner_3d_[i] << endl;
    }

    has_extrinsic_calib_ = true;
  }


  void spin() {
    ros::Rate r(10);
    while (nh_.ok()) {
      publish_3d_markers();

      if (has_extrinsic_calib_) {
        tf_br_.sendTransform(tf::StampedTransform(tform_, ros::Time::now(), parent_tf_.c_str(), tf_.c_str()));
      }

      ros::spinOnce();
      r.sleep();
    }
  }

  void saveYaml() {

    if (has_extrinsic_calib_) {
      // write yaml file before shutdown
      string pkgpath = ros::package::getPath("object_detection_2d");
      string yaml_file_name(pkgpath+"/config/base_top_down_tf.yaml");
      ofstream yaml_file (yaml_file_name.c_str());
      if (yaml_file.is_open()) {
        ROS_INFO("Saving extrinsic calibration parameters in %s", yaml_file_name.c_str());

        yaml_file << "trans: [" << t_(0) << ", "
                                << t_(1) << ", "
                                << t_(2) << "] " << endl;

        tf::Quaternion q = tform_.getRotation();
        yaml_file << "rot: [" << q.x() << ", "
                              << q.y() << ", "
                              << q.z() << ", "
                              << q.w() << "] " << endl;
        yaml_file << "parent: " << parent_tf_ << endl;
        yaml_file << "child: " << tf_ << endl;
        yaml_file.close();

      } else {
        ROS_INFO("IO error. Failed to create yaml file: %s", yaml_file_name.c_str());
      }
    }
  }
};

BoardTracker * g_board_tracker = NULL;
void sigintHandler(int sig) {
  if (g_board_tracker) {
    g_board_tracker->saveYaml();
  }

  ros::shutdown();
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "top_down_calib", ros::init_options::NoSigintHandler);
  BoardTracker board_tracker;
  g_board_tracker = &board_tracker;

  signal(SIGINT, sigintHandler);

  board_tracker.spin();
  return 0;
}