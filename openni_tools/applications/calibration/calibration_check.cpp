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
#include "calibration_check.h"

void printHelp (char** argv)
{
  std::cout << "Visualize the results of the calibration of an OpenNI2 device." << std::endl;
  std::cout << std::endl;
  std::cout << "Usage: " << utl::fs::getBasename(std::string(argv[0])) << " <input_dir> [parameters]" << std::endl;
  std::cout << "<input_dir> must contain a directory named 'ir_rgb' containing images of a checkerboard" << std::endl;
  std::cout << " as well as the calibration file 'rgb_ir_calib.xml' containing calibration between IR" << std::endl;
  std::cout << "and RGB cameras." << std::endl;
  std::cout << std::endl;
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
  
  std::string calibrationDirname = "";
  bool print_help = false;
  
  parseCommandLine(argc, argv, calibrationDirname, print_help);
  
  if (print_help)
  {
    printHelp(argv);
    return 0;
  }
    
  //----------------------------------------------------------------------------
  // Check input parameters
  //----------------------------------------------------------------------------

  std::string calibrationResultDirname = utl::fs::fullfile(calibrationDirname, "openni_calib");
  std::string imageDirname = utl::fs::fullfile(calibrationDirname, "calibration_check");
    
  if (calibrationDirname == "")
  {
    std::cout << "You must provide a directory with images used for calibration." << std::endl;
    return -1;
  }
  
  if (!utl::fs::isDirectory(calibrationDirname))
  {
    std::cout << "Input directory provided not exist or is not a directory (" << calibrationDirname << ")." << std::endl;
    return -1;
  }

  if (!utl::fs::isDirectory(imageDirname))
  {
    std::cout << "Input directory provided does not contain a directory 'calibration_check' (" << calibrationDirname << ")." << std::endl;
    return -1;
  }
  
  //----------------------------------------------------------------------------
  // Load calibration parameters
  //----------------------------------------------------------------------------
  
  std::cout << "Loading calibration parameters..." << std::endl;
  
  cv::Mat K_RGB, K_IR, K_depth;
  cv::Mat d_RGB, d_IR, d_depth;
  cv::Size size_RGB, size_IR, size_depth;
  cv::Mat RGBtoIR_R, RGBtoIR_t;

  // Load IR RGB intrinsics and IR to RGB extrinsics
  if (!utl::ocvcalib::readStereoCalibrationParameters(  utl::fs::fullfile(calibrationResultDirname, "ir_rgb_calib.xml"),
                                                        K_IR, K_RGB, d_IR, d_RGB,
                                                        size_IR, size_RGB, RGBtoIR_R, RGBtoIR_t,
                                                        "IR", "RGB")
      )
    return -1;
  
  // Load depth intrinisic parameters
  if (!utl::ocvcalib::readIntrinsicCalibrationParameters  ( utl::fs::fullfile(calibrationResultDirname, "depth_calib.xml"),
                                                            K_depth, d_depth, size_depth, "depth")
      )
    return -1;

  // Load depth distortion maps
  std::vector<cv::Mat> DM_maps;
  std::vector<float> DM_distances;
  if (!utl::kinect::readDepthDistortionParameters(utl::fs::fullfile(calibrationResultDirname, "depth_distortion.xml"), DM_maps, DM_distances))
  {  
    std::cout << "Could not load depth distortion parameters." << std::endl;
    return -1;
  }
  
  //----------------------------------------------------------------------------
  // Convert calibration to Eigen
  //----------------------------------------------------------------------------

  Eigen::Matrix3f K_depth_eig, K_IR_eig, K_RGB_eig;
  cv::cv2eigen(K_depth, K_depth_eig);
  cv::cv2eigen(K_IR, K_IR_eig);
  cv::cv2eigen(K_RGB, K_RGB_eig);
  
  // Camera poses
  Eigen::Affine3f cameraPose_depth = Eigen::Affine3f::Identity();
  Eigen::Affine3f cameraPose_IR = Eigen::Affine3f::Identity();
  Eigen::Affine3f cameraPose_RGB;
  utl::ocvcalib::cvPose2eigPose<float>(RGBtoIR_R, RGBtoIR_t, cameraPose_RGB);
  cameraPose_RGB = cameraPose_RGB.inverse();
  
  // Depth ideal coordinates
  cv::Mat depthImageIdealCoordinates;
  utl::ocvcalib::getIdealPixelCoordinates(size_depth, K_depth, d_depth, depthImageIdealCoordinates);
  
  //----------------------------------------------------------------------------
  // Read images
  //----------------------------------------------------------------------------
  
  std::cout << "Reading images from " << imageDirname << std::endl;
  
  // Get RGB and IR frame indices
  std::vector<std::string> imageFilenames;
  utl::fs::dir(utl::fs::fullfile(imageDirname, "*.png"), imageFilenames);
  
  std::vector<int> depthFrameIndices;
//   std::vector<int> irFrameIndices;
//   std::vector<int> rgbFrameIndices;
//   std::vector<int> frameIndices;
  
  for (size_t imId = 0; imId < imageFilenames.size(); imId++)
  {
    int curFrameIndex;
    utl::kinect::ImageType curImageType;
    if (!utl::kinect::getImageInfo(imageFilenames[imId], curFrameIndex, curImageType))
      continue;
    
    if (curImageType == utl::kinect::ImageType::DEPTH)
      depthFrameIndices.push_back(curFrameIndex);
  }
    
  if (depthFrameIndices.size() > 0)
    std::cout << depthFrameIndices.size() << " depth frames found." << std::endl;
  else
  {
    std::cout << "No depth frames found." << std::endl;
    return -1;
  }

  //----------------------------------------------------------------------------
  // Save depth RGB calibration results
  //----------------------------------------------------------------------------
  
  std::cout << "Saving depth RGB calibration parameters..." << std::endl;
  
  // Save calibration
  if (!utl::ocvcalib::writeStereoCalibrationParameters  ( utl::fs::fullfile(calibrationResultDirname, "depth_rgb_calib.xml"),
                                                          K_depth, K_RGB,
                                                          d_depth, d_RGB,
                                                          size_depth, size_RGB,
                                                          RGBtoIR_R, RGBtoIR_t,
                                                          "depth", "RGB" )
     )
  {
    return -1;
  }  
  
  //----------------------------------------------------------------------------
  // Visualize calibration results
  //----------------------------------------------------------------------------
  
  std::cout << "Visualizaing calibration results..." << std::endl;
  
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
      
      // Add camera frustums
      utl::pclvis::showCamera(visualizer, K_depth_eig, size_depth.height, size_depth.width, cameraPose_depth, "depth_cam", utl::pclvis::Color(1.0, 0.0, 0.0));
      utl::pclvis::showCamera(visualizer, K_RGB_eig, size_RGB.height, size_RGB.width, cameraPose_RGB, "rgb_cam", utl::pclvis::Color(0.0, 0.0, 1.0));
            
      // Get iterator
      visState.iterator_ = utl::math::clampValueCircular<int>(visState.iterator_, 0, depthFrameIndices.size()-1);
      int frameId = depthFrameIndices[visState.iterator_];
      
      // Add text
      visualizer.addText("Image " + std::to_string(visState.iterator_+1) + " / " + std::to_string(depthFrameIndices.size()), 0, 75, 24, 1.0, 1.0, 1.0);      
      
      // Read images
      cv::Mat imDepth, imIR, imRGB;
      bool depthGood = utl::kinect::readFrame(imageDirname, frameId, utl::kinect::DEPTH, imDepth);
      bool irGood = utl::kinect::readFrame(imageDirname, frameId, utl::kinect::IR, imIR);
      bool rgbGood = utl::kinect::readFrame(imageDirname, frameId, utl::kinect::RGB, imRGB);
      
      if (irGood)
        imIR = utl::ocv::visualizeMatrix(imIR);
      
      // Correct depth distortion
      if (visState.coorectDepth_)
        utl::kinect::correctDepthDistortion(imDepth.clone(), imDepth, DM_maps, DM_distances);
      
      // Create cloud
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      utl::kinect::cvDepth2pclCloud_ideal<pcl::PointXYZRGB>(imDepth, depthImageIdealCoordinates, *cloud);
      
      if (visState.cloudDisplayState_ == VisState::CLOUD)
      {
        utl::pclvis::showPointCloud<pcl::PointXYZRGB>(visualizer, cloud, "cloud", visState.pointSize_, utl::pclvis::Color(0.5, 0.5, 0.5));
        visualizer.addText("Original cloud", 0, 50, 24, 1.0, 1.0, 1.0);
      }
      else if (visState.cloudDisplayState_ == VisState::CLOUD_IR_COLORED)
      {
        if (irGood)
        {
          // Undistort IR image
          cv::Mat imIRUndistorted;
          cv::undistort(imIR.clone(), imIRUndistorted, K_IR, d_IR);
          
          // Color the cloud
          textureMapPointCloud(*cloud, imIRUndistorted, K_IR, d_IR, cameraPose_IR);      
          
          // Show pointcloud
          utl::pclvis::showPointCloudColor<pcl::PointXYZRGB>(visualizer, cloud, "cloud", visState.pointSize_);
        }
        visualizer.addText("IR colored cloud", 0, 50, 24, 1.0, 1.0, 1.0);
      }
      else if (visState.cloudDisplayState_ == VisState::CLOUD_RGB_COLORED)
      {
        if (rgbGood)
        {
          // Undistort IR image
          cv::Mat imRGBUndistorted;
          cv::undistort(imRGB.clone(), imRGBUndistorted, K_IR, d_IR);
          
          // Color the cloud
          textureMapPointCloud(*cloud, imRGBUndistorted, K_RGB, d_RGB, cameraPose_RGB);
          
          // Show pointcloud
          utl::pclvis::showPointCloudColor<pcl::PointXYZRGB>(visualizer, cloud, "cloud", visState.pointSize_);        
        }
        visualizer.addText("RGB colored cloud", 0, 50, 24, 1.0, 1.0, 1.0);
      }

      if (irGood) cv::imshow("IR image", imIR);
      if (depthGood) cv::imshow("Depth image", imDepth / 6000 * std::numeric_limits<unsigned short>::max());
      if (rgbGood) cv::imshow("RGB image", imRGB);
    }
    
    // Spin once
    visualizer.spinOnce();
    boost::this_thread::sleep (boost::posix_time::milliseconds (100));
    cv::waitKey(1);
  }
  
  return 0;
}