// STD includes
#include <iostream>

// Utilities
#include <utilities/filesystem.hpp>
#include <utilities/opencv.hpp>
#include <utilities/opencv_calibration.hpp>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>

// PCL
#include <utilities/pcl_visualization.hpp>

// Project includes
#include "openni_tools.hpp"

// State
struct VisState
{
  VisState ()
    : showDepthOriginal_ (true)
    , showDepthUndistorted_ (false)
    , showDepthIdeal_(false)
    , updateDisplay_ (true)
    , pointSize_(1.0)
  {};
  
  bool showDepthOriginal_;
  bool showDepthUndistorted_;
  bool showDepthIdeal_;
  bool updateDisplay_;
  float pointSize_;
};

// Callback
void keyboard_callback (const pcl::visualization::KeyboardEvent &event, void *cookie)
{
  VisState* visState = reinterpret_cast<VisState*> (cookie);
  
  if (event.keyUp ())
  {    
    std::string key = event.getKeySym ();
//     cout << key << " key pressed!\n";
    
    visState->updateDisplay_ = true;
    
    if ((key == "KP_1") || (key == "KP_End"))
      visState->showDepthOriginal_ = !visState->showDepthOriginal_;
    else if ((key == "KP_2") || (key == "KP_Down"))
      visState->showDepthUndistorted_ = !visState->showDepthUndistorted_;
    else if (key == "KP_3")
      visState->showDepthIdeal_ = !visState->showDepthIdeal_;
    
    // Point size
    else if (key == "KP_Add")
      visState->pointSize_ += 1.0;
    else if (key == "KP_Subtract")
      visState->pointSize_ = std::max(0.0, visState->pointSize_ - 1.0);    
    
    else
      visState->updateDisplay_ = false;
  }
}


// //////////////////////////////////////////////////////////////////////////////
// // Create lookup table for rectification
// void CreateUndistortLookupTable (const cv::Mat &K, const cv::Mat &d, const cv::Size &im_size )
// {
//   const int w = im_size.width;
//   const int h = im_size.height
// 
//   // Get undistorted pixel coordinates
//   cv::Mat map1, map2;
//   cv::initUndistortRectifyMap(K, d, cv::Mat(), K, im_size, CV_32FC1, map1, map2);
//   
//   // Calculate bilinear interpolation coefficients
//   
//   
//   for( int r = 0; r < h; ++r) {
//     for( int c = 0; c < w; ++c) {
//       // Remap
//       const Eigen::Vector3d p_o = R_onKinv * Eigen::Vector3d(c,r,1);
//       Eigen::Vector2d p_warped = cam_from.Project(p_o);
// 
//       bool outside = (p_warped[0] > w - 1.0 || p_warped[0] < 0.0 || p_warped[1] > h - 1.0 || p_warped[1] < 0.0);
//       
//       // Clamp to valid image coords. This will cause out of image
//       // data to be stretched from nearest valid coords with
//       // no branching in rectify function.
//       p_warped[0] = std::min(std::max(0.0, p_warped[0]), w - 1.0 );
//       p_warped[1] = std::min(std::max(0.0, p_warped[1]), h - 1.0 );
// 
//       // Truncates the values for the left image
//       int u  = (int) p_warped[0];
//       int v  = (int) p_warped[1];
//       float su = p_warped[0] - (double)u;
//       float sv = p_warped[1] - (double)v;
// 
//       // Fix pixel access for last row/column to ensure all are in bounds
//       if(u == (w-1)) {
//         u -= 1;
//         su = 1.0;
//       }
//       if(v == (w-1)) {
//         v -= 1;
//         sv = 1.0;
//       }
// 
//       if (outside)
//       {
//         BilinearLutPoint p;
//         p.idx0 = u + v*w;
//         p.idx1 = u + v*w + w;
//         p.w00  = 0;
//         p.w01  = 0;
//         p.w10  = 0;
//         p.w11  = 0;
//         lut.SetPoint( r, c, p );          
//       }
//       else
//       {
//         // Pre-compute the bilinear interpolation weights
//         BilinearLutPoint p;
//         p.idx0 = u + v*w;
//         p.idx1 = u + v*w + w;
//         p.w00  = (1-su)*(1-sv);
//         p.w01  =    su *(1-sv);
//         p.w10  = (1-su)*sv;
//         p.w11  =     su*sv;
//         lut.SetPoint( r, c, p );
//       }
//     }
//   }
// }

namespace utl
{
  namespace ocvcalib
  {
    
  }
}



int main( int argc, char** argv )
{  
  //----------------------------------------------------------------------------
  // Load image and calibration parameters
  //----------------------------------------------------------------------------
  
  // Image
  cv::Mat imDepth;
  utl::kinect::readFrame("../images", 56, utl::kinect::DEPTH, imDepth);  
  
  // Calibration
  cv::Mat K_depth, K_depth_orig;
  cv::Mat d_depth;
  cv::Size size_depth;
  
  if (!utl::ocvcalib::readIntrinsicCalibrationParameters ( utl::fs::fullfile("../images/openni_calib", "depth_calib.xml"),
                                                      K_depth, d_depth, size_depth, "depth"))
  {
    return -1;
  }  
  
  //----------------------------------------------------------------------------
  // Undistort OpenCV
  //----------------------------------------------------------------------------
  
  cv::Mat imDepthUndistortedCV;
  cv::undistort(imDepth, imDepthUndistortedCV, K_depth, d_depth, K_depth);
  
  //----------------------------------------------------------------------------
  // Undistort discontinuity free
  //----------------------------------------------------------------------------
  
  // Get undistortion maps
  cv::Mat map_x, map_y;
  cv::initUndistortRectifyMap(K_depth, d_depth, cv::Mat(), K_depth, size_depth, CV_32FC1, map_x, map_y);
  
  cv::Mat imDepthUndistorted = utl::kinect::undistortDepthImage(imDepth, map_x, map_y, 50.0f);
  
//   // Get discontinuity map
//   cv::Mat validPixels;
//   cv::Mat tmp;
//   imDepth.convertTo(tmp, CV_8U, 1 /255.0f);
//   cv::threshold(tmp, validPixels, 0, 255, CV_THRESH_BINARY);
//   
//   // Get discontinuity map
//   cv::Mat depthDisc;
//   utl::kinect::getDepthDiscontinuities(imDepth, depthDisc, 50.0f);
//   
//   cv::bitwise_and(validPixels, 255 - depthDisc, validPixels);
//     
//   // Undistort
//   cv::Mat imDepthUndistorted = utl::ocvcalib::undistort(imDepth, map_x, map_y, validPixels);
//   
//   std::cout << imDepthUndistorted.size() << std::endl;
//   std::cout << validPixels.size() << std::endl;
//   
//   cv::imshow("Disc undist", validPixels);
//   cv::imshow("Depth undistorted", imDepthUndistorted * 30);
//   cv::waitKey();

  //----------------------------------------------------------------------------
  // Create clouds
  //----------------------------------------------------------------------------

  pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud (new pcl::PointCloud<pcl::PointXYZ>);
  utl::kinect::cvDepth2pclCloud_undistorted<pcl::PointXYZ>(imDepth, K_depth, *originalCloud);
  
  cv::Mat idealPixelCoordinates;
  utl::ocvcalib::getIdealPixelCoordinates(imDepth.size(), K_depth, d_depth, idealPixelCoordinates);
  pcl::PointCloud<pcl::PointXYZ>::Ptr idealCloud (new pcl::PointCloud<pcl::PointXYZ>);
  utl::kinect::cvDepth2pclCloud_ideal<pcl::PointXYZ>(imDepth, idealPixelCoordinates, *idealCloud);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr undistortedCloud (new pcl::PointCloud<pcl::PointXYZ>);
  utl::kinect::cvDepth2pclCloud_undistorted<pcl::PointXYZ>(imDepthUndistorted, K_depth, *undistortedCloud);
  
  //----------------------------------------------------------------------------
  // Visualize
  //----------------------------------------------------------------------------
  
  cv::imshow("Original depth", imDepth * 30);
  cv::imshow("Undistorted depth", imDepthUndistorted * 30);
  
  VisState visState;
  pcl::visualization::PCLVisualizer visualizer;
  visualizer.setCameraPosition (  0.0, 0.0, -1.0,   // camera position
                                  0.0, 0.0, 1.0,   // viewpoint
                                  0.0, -1.0, 0.0,   // normal
                                  0.0);            // viewport
  visualizer.setBackgroundColor (utl::pclvis::bgColor.r, utl::pclvis::bgColor.g, utl::pclvis::bgColor.b);
  visualizer.registerKeyboardCallback(keyboard_callback, (void*)(&visState));  
  
  visState.updateDisplay_ = true;
  while (!visualizer.wasStopped())
  {
    // Update display if needed
    if (visState.updateDisplay_)
    {
      // First remove everything
      visualizer.removeAllPointClouds();
      visualizer.removeAllShapes();
      visualizer.removeAllCoordinateSystems();
      visState.updateDisplay_ = false;
            
      if (visState.showDepthOriginal_)
        utl::pclvis::showPointCloud<pcl::PointXYZ>(visualizer, originalCloud, "orig", visState.pointSize_, utl::pclvis::Color(1.0, 0.0, 0.0));

      if (visState.showDepthUndistorted_)
        utl::pclvis::showPointCloud<pcl::PointXYZ>(visualizer, undistortedCloud, "undistort", visState.pointSize_, utl::pclvis::Color(0.0, 1.0, 0.0));
      
      if (visState.showDepthIdeal_)
        utl::pclvis::showPointCloud<pcl::PointXYZ>(visualizer, idealCloud, "ideal", visState.pointSize_, utl::pclvis::Color(0.0, 0.0, 1.0));      
    }
    
    // Spin once
    visualizer.spinOnce();
    boost::this_thread::sleep (boost::posix_time::milliseconds (100));
    cv::waitKey(1);    
  }
  
  return 0;  
}