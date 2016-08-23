#ifndef OPENNI2_CALIBRATION_H
#define OPENNI2_CALIBRATION_H

// Utilities
#include <namaris/utilities/pcl_visualization.hpp>
#include <namaris/utilities/opencv_calibration.hpp>

// Project includes
#include <openni_tools/openni_tools.hpp>

////////////////////////////////////////////////////////////////////////////////
// Generate a default calibration board object
inline
utl::ocvcalib::CalibTarget defaultCalibTarget ()
{
  utl::ocvcalib::CalibTarget board;
  
  board.size_.width   = 7;
  board.size_.height  = 5;
  board.squareSize_        = 0.038f;   // m
  
  return board;
}

////////////////////////////////////////////////////////////////////////////////
// Project a 3D polygon in world coordinates to image plane and return a mask of
// pixels inside the projected polygon
cv::Mat get3DpolygonImageMask ( const pcl::PointCloud<pcl::PointXYZ> &polygon,
                                const cv::Mat &K, const cv::Mat &d, const cv::Size &image_size, const Eigen::Affine3f &camera_pose,
                                const float scale = 1.0f);

////////////////////////////////////////////////////////////////////////////////
/** \brief Color points in a pointcloud using an image from a known camera.
  *  \param[in,out] cloud   input pointcloud
  *  \param[in] image       input image
  *  \param[in] K           camera matrix for the camera used to obtain the image
  *  \param[in] d           distortion parameters of the camera used to obtain the image
  *  \param[in] camera_pose pose of the camera in the pointcloud coordinate system
  */
void textureMapPointCloud ( pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                            const cv::Mat &image,
                            const cv::Mat &K, const cv::Mat &d,
                            const Eigen::Affine3f &camera_pose
                          );

////////////////////////////////////////////////////////////////////////////////
// Scale IR high resolution intrnisc parameters to low resolution
void IRIntrinsicParametersHighResToLowRes(const cv::Mat& K_high_res, cv::Mat& K_low_res);

////////////////////////////////////////////////////////////////////////////////
// Project detected target corners to target plane
void showTargetPoints ( pcl::visualization::PCLVisualizer &visualizer,
                        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &calibTargetPoints,
                        const Eigen::Affine3f &targetPose,
                        const std::string &id_prefix = "target",
                        const utl::pclvis::Color color = utl::pclvis::Color()
                      );

////////////////////////////////////////////////////////////////////////////////   
// Visualize a depth multiplier map
cv::Mat visualizeDepthMultiplierMap  (const cv::Mat &dm_map);

#endif  // OPENNI2_CALIBRATION_H