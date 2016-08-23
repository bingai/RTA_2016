// STD includes
#include <iostream>

// Utilities
#include <namaris/utilities/filesystem.hpp>
#include <namaris/utilities/math.hpp>
#include <namaris/utilities/opencv.hpp>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>

// Project includes
#include "openni_tools/openni_tools.hpp"
#include "openni_tools/calibration/openni_calibration.h"
#include "depth_ir_calibration.h"

void printHelp (char** argv)
{
  std::cout << "Manually (visually) align depth to IR images using arrow keys, traverse frames using ',' and '.' keys, "
      " save new depth intrinsics using space key, quit application using 'q'." << std::endl;
  std::cout << std::endl;
  std::cout << "Usage: " << utl::fs::getBasename(std::string(argv[0])) << " <input_dir> [parameters]" << std::endl;
  std::cout << "<input_dir> must contain a directory named 'calibration_check' containing frames of a checkerboard"
      " from depth & IR cameras taken with openni_capture_util." << std::endl;
  std::cout << "<input_dir> must also contain a folder called 'openni_calib' containing calibration file 'ir_calib.xml',"
      " with the intrinsics of the IR camera." << std::endl;
  std::cout << "Parameters:" << std::endl;
  std::cout << "      -h:               show this message" << std::endl;
}

void parseCommandLine ( int argc, char** argv,
                        std::string &input_dir,
                        bool &print_help
                      )
{   
  // Check parameters
  for (size_t i = 1; i < static_cast<size_t>(argc); i++)
  {
    std::string curParameter (argv[i]);
    
    if (curParameter == "-h")
      print_help = true;
        
    else if (curParameter[0] != '-' && input_dir.empty())
      input_dir = curParameter;
    
    else 
      std::cout << "Unknown parameter '" << curParameter << "'" << std::endl;
  }
}

int main( int argc, char** argv )
{
  //----------------------------------------------------------------------------
  // Parse command line arguments
  //----------------------------------------------------------------------------
  
  std::string calibrationImageDirname = "";
  bool print_help = false;
  
  parseCommandLine(argc, argv, calibrationImageDirname, print_help);
  
  if (print_help)
  {
    printHelp(argv);
    return 0;
  }
    
  //----------------------------------------------------------------------------
  // Check input parameters
  //----------------------------------------------------------------------------

  std::string calibrationResultDirname = utl::fs::fullfile(calibrationImageDirname, "openni_calib");
  std::string imageDirname = utl::fs::fullfile(calibrationImageDirname, "calibration_check");
    
  if (calibrationImageDirname == "")
  {
    std::cout << "You must provide a directory with images used for calibration." << std::endl;
    return -1;
  }
  
  if (!utl::fs::isDirectory(calibrationImageDirname))
  {
    std::cout << "Input directory provided does not exist or is not a directory (" << calibrationImageDirname << ")." << std::endl;
    return -1;
  }

  if (!utl::fs::isDirectory(imageDirname))
  {
    std::cout << "Input directory provided does not contain a directory 'calibration_check' (" << calibrationImageDirname << ")." << std::endl;
    return -1;
  }
  
  //----------------------------------------------------------------------------
  // Load calibration parameters
  //----------------------------------------------------------------------------
  
  cv::Mat K_IR, K_depth, K_depth_orig;
  cv::Mat d_IR, d_depth;
  cv::Size size_IR, size_depth;

  if (!utl::ocvcalib::readIntrinsicCalibrationParameters ( utl::fs::fullfile(calibrationResultDirname, "ir_calib.xml"),
                                                      K_IR, d_IR, size_IR, "IR"))
  {
    return -1;
  }
  
  
  
  // Convert IR calibration to depth calibration
  size_depth = cv::Size(640, 480);
  if (size_IR == cv::Size(1280, 1024))
    IRIntrinsicParametersHighResToLowRes(K_IR, K_depth_orig);
  else
    K_depth_orig = K_IR;
  d_depth = d_IR;
  
  //----------------------------------------------------------------------------
  // Read depth and RGB frame pairs
  //----------------------------------------------------------------------------
  
  std::cout << "Reading images from " << imageDirname << std::endl;
  
  // Get RGB and IR frame indices
  std::vector<std::string> imageFilenames;
  utl::fs::dir(utl::fs::fullfile(imageDirname, "*.png"), imageFilenames);
  
  std::vector<int> irFrameIndices;
  std::vector<int> depthFrameIndices;
  std::vector<int> depth_ir_frames;
  
  for (size_t imId = 0; imId < imageFilenames.size(); imId++)
  {
    int curFrameIndex;
    utl::kinect::ImageType curImageType;
    if (!utl::kinect::getImageInfo(imageFilenames[imId], curFrameIndex, curImageType)) {
      continue;
    }
    
    if (curImageType == utl::kinect::ImageType::IR)
      irFrameIndices.push_back(curFrameIndex);
    else if (curImageType == utl::kinect::ImageType::DEPTH)
      depthFrameIndices.push_back(curFrameIndex);
  }
  
  // Get depth IR frame pairs
  for (auto frameIt = irFrameIndices.begin(); frameIt != irFrameIndices.end(); frameIt++)
  {
    if (std::find(depthFrameIndices.begin(), depthFrameIndices.end(), *frameIt) != depthFrameIndices.end())
      depth_ir_frames.push_back(*frameIt);
  }
  
  if (depth_ir_frames.size() > 0)
    std::cout << depth_ir_frames.size() << " depth IR frame pairs found." << std::endl;
  else
  {
    std::cout << "No depth IR frame pairs found." << std::endl;
    return -1;
  }

  //----------------------------------------------------------------------------
  // Calibrate
  //----------------------------------------------------------------------------
  
  double dx = 0;
  double dy = 0;
  double step = -0.1;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);  
  
  //----------------------------------------------------------------------------
  // Visualize results
  //----------------------------------------------------------------------------
  
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
      
      // Update iterator
      visState.iterator_ = utl::math::clampValueCircular<int>(visState.iterator_, 0, depth_ir_frames.size()-1);
      int frameId = depth_ir_frames[visState.iterator_];
      
      // Load frames
      cv::Mat imDepth, imIR;
      utl::kinect::readFrame(imageDirname, frameId, utl::kinect::DEPTH, imDepth);
      utl::kinect::readFrame(imageDirname, frameId, utl::kinect::IR, imIR);
      
      // Check expected image sizes
      if (imDepth.size() != size_depth || imIR.size() != size_IR)
      {
        std::cout << "Calibration image size and loaded image size are different." << std::endl;
        std::cout << "Depth expected: "  << size_depth << std::endl;
        std::cout << "Depth loaded:   "  << imDepth.size() << std::endl;
        std::cout << "IR expected:    "  << size_IR << std::endl;
        std::cout << "IR loaded:      "  << imIR.size() << std::endl;
      }
      
      // Convert IR for visualization
      cv::Mat imIRNormalized;
      imIR.convertTo(imIRNormalized, CV_32F);
      imIRNormalized = imIRNormalized / 200;
      
      // Update depth camera matrix
      dx += static_cast<double>(visState.dx_) * step;
      dy += static_cast<double>(visState.dy_) * step;
      
      K_depth = K_depth_orig.clone();
      K_depth.at<double>(0,2) += dx;
      K_depth.at<double>(1,2) += dy;
      
      //------------------------------------------------------------------------
      // Update cloud
      //------------------------------------------------------------------------
      
      if (visState.cloudDisplayState_ != VisState::NOTHING)
      {
        // Undistort IR image
        cv::Mat imIRNormalizedUndistorted;
        cv::undistort(imIRNormalized, imIRNormalizedUndistorted, K_IR, d_IR);
        
  //       // Correct depth
  //       if (visState.coorectDepth_)
  //       {
  //         utl::kinect::correctDepthDistortion(imDepth.clone(), imDepth, DM_maps, DM_distances);        
  //       }        
        
        // Construct cloud
        cv::Mat depthImageIdealCoordinates;
        utl::ocvcalib::getIdealPixelCoordinates(size_depth, K_depth, d_depth, depthImageIdealCoordinates);
        utl::kinect::cvDepth2pclCloud_ideal<pcl::PointXYZRGB>(imDepth, depthImageIdealCoordinates, *cloud);
        
        // Display stuff
        if (visState.cloudDisplayState_ == VisState::CLOUD)
        {
          utl::pclvis::showPointCloud<pcl::PointXYZRGB>(visualizer, cloud, "cloud", visState.pointSize_, utl::pclvis::Color(0.5, 0.5, 0.5));
        }
        
        else if (visState.cloudDisplayState_ == VisState::CLOUD_IR_COLORED)
        {
          // Color the cloud
          cv::Mat tmp;
          imIRNormalizedUndistorted.convertTo(tmp, CV_8U, 255);
          textureMapPointCloud(*cloud, tmp, K_IR, d_IR, Eigen::Affine3f::Identity());      
          
          // Show pointcloud
          utl::pclvis::showPointCloudColor<pcl::PointXYZRGB>(visualizer, cloud, "cloud", visState.pointSize_);        
        }
      }
      
      // Add text
      visualizer.addText("Frame " + std::to_string(visState.iterator_+1) + " / " + std::to_string(depth_ir_frames.size()), 0, 100, 24, 1.0, 1.0, 1.0);
      visualizer.addText("dx = " + utl::str::to_string(dx, 1), 0, 75, 24, 1.0, 1.0, 1.0);
      visualizer.addText("dy = " + utl::str::to_string(dy, 1), 0, 50, 24, 1.0, 1.0, 1.0);
      
      //------------------------------------------------------------------------
      // Update image
      //------------------------------------------------------------------------

      //------------------------------------------------------------------------
      // Prepare depth image
      
      // Normalize depth image for display
      cv::Mat imDepthNormalized;
      imDepth.convertTo(imDepthNormalized, CV_32FC1);
      
      // Find mask of valid depth pixels
      cv::Mat validDepthMask;
      cv::threshold(imDepthNormalized, validDepthMask, 400, 1, CV_THRESH_BINARY );
      validDepthMask.convertTo(validDepthMask, CV_8U, 255);

      // Find minimum valid depth value
      double minDepthVal, maxDepthVal;
      cv::minMaxIdx(imDepthNormalized, &minDepthVal, &maxDepthVal, 0, 0, validDepthMask);
          
      // Noramlize depth image
      float minDepthValDisplayed = minDepthVal - 500;
      float maxDepthValDisplayed = minDepthVal + 1000;
      
      imDepthNormalized = (imDepthNormalized - minDepthValDisplayed) / (maxDepthValDisplayed - minDepthValDisplayed);
      imDepthNormalized = 1 - imDepthNormalized;
      
      cv::Mat invalidDepthMask = 255 - validDepthMask;
      cv::Mat zeroMat = cv::Mat::zeros(imDepth.size(), CV_32F);
      zeroMat.copyTo(imDepthNormalized, invalidDepthMask);
//       
      // Shift depth image
      cv::Mat imDepthShifted;
      cv::undistort(imDepthNormalized, imDepthShifted, K_depth, cv::Mat::zeros(1, 5, CV_64F), K_depth_orig);
      
      //------------------------------------------------------------------------
      // Resize images for display

      cv::Mat imIRNormalizedCropped;
      
      if (size_IR == cv::Size(1280, 1024))
      {
        int y_excess = size_IR.height - size_depth.height*2;
        imIRNormalizedCropped = cv::Mat(imIRNormalized, cv::Rect(0, y_excess/2 + 1, size_depth.width * 2, size_depth.height * 2));
        cv::resize(imDepthShifted, imDepthShifted, cv::Size(), 2.0, 2.0);
      }
      else
      {
        imIRNormalized.copyTo(imIRNormalizedCropped);
        cv::resize(imIRNormalizedCropped, imIRNormalizedCropped, cv::Size(), 2.0, 2.0);
        cv::resize(imDepthShifted, imDepthShifted, cv::Size(), 2.0, 2.0);
      }
            
      //------------------------------------------------------------------------
      // Overlay images      

      // Generate image
      cv::Mat alignmentDisplay;
      std::vector<cv::Mat> channels (3);
      channels[0] = imDepthShifted;
      channels[1] = cv::Mat::zeros(imDepthShifted.size(), CV_32F);
      channels[2] = imIRNormalizedCropped;
      cv::merge(channels, alignmentDisplay);
            
      cv::imshow("Image alignment", alignmentDisplay);
      cv::imshow("depth", imDepthShifted);
      
      //------------------------------------------------------------------------
      // Save depth calibration
      //------------------------------------------------------------------------      
      
      if (visState.saveResults_)
      {      
        std::string calibOutFilename = utl::fs::fullfile(calibrationResultDirname, "depth_calib.xml");
        if (!utl::ocvcalib::writeIntrinsicCalibrationParameters ( calibOutFilename,
                                                                K_depth, d_depth, size_depth, "depth"))
        {
          std::cout << "Could not save depth calibration parameters saved to '" << calibOutFilename << "'" << std::endl;
        }
        else
        {
          std::cout << "Depth calibration parameters saved to '" << calibOutFilename << "'" << std::endl; 
        }
      }
    }
    
    // Spin once
    visualizer.spinOnce();
    boost::this_thread::sleep (boost::posix_time::milliseconds (10));
    cv::waitKey(1);
    
  }
  
  return 0; 
 
}