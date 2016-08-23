// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Utilities
#include <namaris/utilities/opencv.hpp>
#include <namaris/utilities/pointcloud.hpp>

// Project includes
#include <openni_tools/openni_tools.hpp>
#include <openni_tools/calibration/openni_calibration.h>

////////////////////////////////////////////////////////////////////////////////
cv::Mat get3DpolygonImageMask ( const pcl::PointCloud<pcl::PointXYZ> &polygon,
                                const cv::Mat &K, const cv::Mat &d, const cv::Size &image_size, const Eigen::Affine3f &camera_pose,
                                const float scale )
{
  // Scale polygon
  pcl::PointCloud<pcl::PointXYZ> polygon_scaled;
  
  if (scale == 1.0f)
    pcl::copyPointCloud(polygon, polygon_scaled);
  else
    utl::cloud::scalePointCloud<pcl::PointXYZ>(polygon, scale, polygon_scaled);
 
  // Convert to OpenCV polygon
  std::vector<cv::Point3f> polygon_cv;
  utl::kinect::pclCloud2cvCloud<pcl::PointXYZ>(polygon_scaled, polygon_cv);
  
  return utl::ocvcalib::get3DpolygonImageMask(polygon_cv, K, d, image_size, camera_pose);
}

////////////////////////////////////////////////////////////////////////////////
void textureMapPointCloud ( pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                            const cv::Mat &image,
                            const cv::Mat &K, const cv::Mat &d,
                            const Eigen::Affine3f &camera_pose
                          )
{
  //----------------------------------------------------------------------------
  // Check input parameters
  
  cv::Mat image_tmp;
  
  if (image.type() == CV_8UC3)
    image_tmp = image.clone();
  else if (image.type() == CV_8UC1)
    cv::cvtColor(image, image_tmp, CV_GRAY2BGR);
  else
  {
    std::cout << "[textureMapPointCloud] image must be of type CV_8UC1 or CV_8UC3." << std::endl;
    abort();
  }
  
  //----------------------------------------------------------------------------
  // Project pointcloud points to image frame
  
  std::vector<cv::Point3f> cloud_cv;
  std::vector<cv::Point2f> image_points;
  utl::kinect::pclCloud2cvCloud<pcl::PointXYZRGB>(cloud, cloud_cv);
  
  // Convert camera pose to OpenCV
  cv::Mat R, rvec, tvec;
  utl::ocvcalib::eigPose2cvPose(camera_pose.inverse(), R, tvec);
  cv::Rodrigues(R, rvec);
  
  // Project points
  cv::projectPoints(cloud_cv, rvec, tvec, K, d, image_points);
  
  //----------------------------------------------------------------------------
  // Interploate values and set values for the pointcloud
  
  std::cout << (image_tmp.type() == CV_8UC1) << std::endl;
  
  for (size_t pointId = 0; pointId < cloud.size(); pointId++)
  {
    if (  std::isfinite(image_points[pointId].x) && image_points[pointId].x >= 0 && image_points[pointId].x <= image.cols - 1   &&
          std::isfinite(image_points[pointId].y) && image_points[pointId].y >= 0 && image_points[pointId].y <= image.rows - 1  )
    {
      int imPointX = static_cast<int>(image_points[pointId].x);
      int imPointY = static_cast<int>(image_points[pointId].y);

//       cloud.points[pointId].r = image_tmp.at<uchar>(imPointY, imPointX);
//       cloud.points[pointId].g = image_tmp.at<uchar>(imPointY, imPointX);
//       cloud.points[pointId].b = image_tmp.at<uchar>(imPointY, imPointX);
      
      cloud.points[pointId].b = image_tmp.at<cv::Vec3b>(imPointY, imPointX).val[0];
      cloud.points[pointId].g = image_tmp.at<cv::Vec3b>(imPointY, imPointX).val[1];
      cloud.points[pointId].r = image_tmp.at<cv::Vec3b>(imPointY, imPointX).val[2];      
    }
  }  
}

////////////////////////////////////////////////////////////////////////////////
void IRIntrinsicParametersHighResToLowRes(const cv::Mat& K_high_res, cv::Mat &K_low_res)
{  
  // Check input
  if (K_high_res.type() != CV_64F)
  {
    std::cout << "[IRIntrinsicParametersHighResToLowRes] input camera matrix must be of type CV_64F." << std::endl;
    std::abort();
  }
    
  double sx = 640.0 / 1280.0;
  double sy = 480.0 / 1024.0;
  
  K_low_res = K_high_res.clone();

  K_low_res.at<double>(0, 0) *= sx;
  K_low_res.at<double>(0, 2) *= sx;
  K_low_res.at<double>(1, 1) *= sx;
  K_low_res.at<double>(1, 2) *= sy;
}

////////////////////////////////////////////////////////////////////////////////
void showTargetPoints ( pcl::visualization::PCLVisualizer &visualizer,
                        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &calibTargetPoints,
                        const Eigen::Affine3f &targetPose,
                        const std::string &id_prefix,
                        const utl::pclvis::Color color
                      )
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr targetTransformed (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*calibTargetPoints, *targetTransformed, targetPose);
  
  for (size_t i = 0; i < targetTransformed->size(); i++)
  {
    std::string id = id_prefix + "_" + std::to_string(i);
    visualizer.addSphere(targetTransformed->points[i], 0.01, id);
    utl::pclvis::setShapeRenderProps(visualizer, id, color);
    if (i == 0)
      visualizer.addText3D(std::to_string(0), targetTransformed->points[i], 0.03, 1.0, 1.0, 1.0, id_prefix + "_first");
    if (i == targetTransformed->size()-1)
      visualizer.addText3D(std::to_string(targetTransformed->size()-1), targetTransformed->points[i], 0.03, 1.0, 1.0, 1.0, id_prefix + "_last");
    
  }
}

////////////////////////////////////////////////////////////////////////////////   
cv::Mat visualizeDepthMultiplierMap  (const cv::Mat &dm_map)
{  
  float minimumMaxVal = 0.01;
  cv::Mat depthMultiplierVis = dm_map.clone();
  
  // Replace 0 weights with 1
  cv::Mat invalidDmMask;
  cv::threshold(dm_map, invalidDmMask, 0, 1, CV_THRESH_BINARY_INV );
  invalidDmMask.convertTo(invalidDmMask, CV_8U, 255);
  
  if (cv::countNonZero(invalidDmMask) > 0)
  {
    std::cout << "[visualizeDepthMultiplierMap] 0 multipliers replaced with 1." << std::endl;
    cv::Mat tmp = cv::Mat::ones(dm_map.size(), CV_32F);
    tmp.copyTo(depthMultiplierVis, invalidDmMask);
  }
  
  // Find maximum depth multiplier value
  depthMultiplierVis = depthMultiplierVis - 1;
  double min, max;
  cv::minMaxIdx(depthMultiplierVis, &min, &max);
  float maxVal = std::max(std::abs(min), std::abs(max)); 
  maxVal = std::max(maxVal, minimumMaxVal);    
  
  // Normalize depth multiplier image for display
  depthMultiplierVis = (depthMultiplierVis + maxVal) / (maxVal*2);
    
  // Apply colormap
  depthMultiplierVis.convertTo(depthMultiplierVis,CV_8U, 255, 0);
  cv::Mat b2rColormapLUT = utl::ocv::b2rColormapLUT();
  utl::ocv::applyColormap(depthMultiplierVis, depthMultiplierVis, b2rColormapLUT);
  
  return depthMultiplierVis;
}