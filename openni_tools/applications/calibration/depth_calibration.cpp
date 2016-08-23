// STD includes
#include <iostream>

// Utilities
#include <namaris/utilities/filesystem.hpp>
#include <namaris/utilities/geometry.hpp>
#include <namaris/utilities/pointcloud.hpp>
#include <namaris/utilities/opencv.hpp>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>

// OMP
#include <omp.h>

//
#include "openni_tools/openni_tools.hpp"
#include "openni_tools/calibration/openni_calibration.h"
#include "depth_calibration.h"

void printHelp (char** argv)
{
  std::cout << "Create depth calibration lookup table from provided depth images & depth camera intrinsics." << std::endl;
  std::cout << std::endl;
  std::cout << "Usage: " << utl::fs::getBasename(std::string(argv[0])) << " <input_dir> [parameters]" << std::endl;
  std::cout << "<input_dir> must contain a folder called 'frames' containing depth and ir frames of a wall-mounted calibration board"
      " taken with the openni_capture_util." << std::endl;
  std::cout << "<input_dir> must also contain a folder called 'openni_calib' containing "
      "calibration file 'depth_calib.xml' and 'ir_calib.xml,"
      " with the intrinsics of the depth and IR cameras, respectively." << std::endl;
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
  std::string depth_IR_dirname = utl::fs::fullfile(calibrationImageDirname, "frames");
    
  if (calibrationImageDirname == "")
  {
    std::cout << "You must provide a directory with images used for calibration." << std::endl;
    return -1;
  }
  
  if (!utl::fs::isDirectory(calibrationImageDirname))
  {
    std::cout << "Input directory provided not exist or is not a directory (" << calibrationImageDirname << ")." << std::endl;
    return -1;
  }

  if (!utl::fs::isDirectory(depth_IR_dirname))
  {
    std::cout << "Input directory provided does not contain a directory 'frames' (" << calibrationImageDirname << ")." << std::endl;
    return -1;
  }
  
  //----------------------------------------------------------------------------
  // Load calibration parameters
  //----------------------------------------------------------------------------
  
  cv::Mat K_IR, K_depth;
  cv::Mat d_IR, d_depth;
  cv::Size size_IR, size_depth;
  //utl::ocvcalib::CalibTarget calibTarget = defaultCalibTarget();
  utl::ocvcalib::CalibTarget calibTarget;
  calibTarget.size_.width   = 14;
  calibTarget.size_.height  = 9;
  calibTarget.squareSize_   = 0.05275;   // m
  
  if (!utl::ocvcalib::readIntrinsicCalibrationParameters  ( utl::fs::fullfile(calibrationResultDirname, "depth_calib.xml"),
                                                            K_depth, d_depth, size_depth, "depth" )
    )
  {
    return -1;
  }

  if (!utl::ocvcalib::readIntrinsicCalibrationParameters  ( utl::fs::fullfile(calibrationResultDirname, "ir_calib.xml"),
                                                            K_IR, d_IR, size_IR, "IR" )
    )
  {
    return -1;
  }
  
   
  float calibTargetScale = 6.0f;
   
  //----------------------------------------------------------------------------
  // Read depth and IR frame pairs
  //----------------------------------------------------------------------------
  
  std::cout << "Reading images from " << depth_IR_dirname << std::endl;
  
  // Get RGB and IR frame indices
  std::vector<std::string> imageFilenames;
  utl::fs::dir(utl::fs::fullfile(depth_IR_dirname, "*.png"), imageFilenames);
  
  std::vector<int> IRFrameIndices;
  std::vector<int> depthFrameIndices;
  std::vector<int> depth_IR_frames;
  
  for (size_t imId = 0; imId < imageFilenames.size(); imId++)
  {
    int curFrameIndex;
    utl::kinect::ImageType curImageType;
    if (!utl::kinect::getImageInfo(imageFilenames[imId], curFrameIndex, curImageType))
      continue;
    
    if (curImageType == utl::kinect::ImageType::IR)
      IRFrameIndices.push_back(curFrameIndex);
    else if (curImageType == utl::kinect::ImageType::DEPTH)
      depthFrameIndices.push_back(curFrameIndex);
  }
  
  // Get RGB IR frame pairs
  for (auto IRFrameIt = IRFrameIndices.begin(); IRFrameIt != IRFrameIndices.end(); IRFrameIt++)
  {
    if (std::find(depthFrameIndices.begin(), depthFrameIndices.end(), *IRFrameIt) != depthFrameIndices.end())
      depth_IR_frames.push_back(*IRFrameIt);
  }
  
//   depth_IR_frames[0] = depth_IR_frames[14];
//   depth_IR_frames.resize(1);
  
  if (depth_IR_frames.size() > 0)
    std::cout << depth_IR_frames.size() << " depth IR frame pairs found." << std::endl;
  else
  {
    std::cout << "No depth IR frame pairs found." << std::endl;
    return -1;
  }

  //----------------------------------------------------------------------------
  // Prepare calibration data
  //----------------------------------------------------------------------------
    
  // Camera matrices
  Eigen::Matrix3f K_depth_eig;
  Eigen::Matrix3f K_IR_eig;
    
  cv::cv2eigen(K_depth, K_depth_eig);
  cv::cv2eigen(K_IR, K_IR_eig);
  
  // Camera poses
  Eigen::Affine3f cameraPose_depth = Eigen::Affine3f::Identity();
  Eigen::Affine3f cameraPose_IR = Eigen::Affine3f::Identity();
  
  // Target points 3D
  std::vector<cv::Point3f> calibTargetPoints3D;
  utl::ocvcalib::generateTargetPoints3D(calibTarget, calibTargetPoints3D);
  pcl::PointCloud<pcl::PointXYZ>::Ptr calibTargetPoints3D_pcl (new pcl::PointCloud<pcl::PointXYZ>);
  utl::kinect::cvCloud2pclCloud<pcl::PointXYZ>(calibTargetPoints3D, *calibTargetPoints3D_pcl);

  // Target bounding box points 3D
  std::vector<cv::Point3f> calibTargetBBXPoints3D;
  utl::ocvcalib::generateTargetBoundingBoxPoints3D(calibTarget, calibTargetBBXPoints3D);
  pcl::PointCloud<pcl::PointXYZ>::Ptr calibTargetBBXPoints3D_pcl (new pcl::PointCloud<pcl::PointXYZ>);
  utl::kinect::cvCloud2pclCloud<pcl::PointXYZ>(calibTargetBBXPoints3D, *calibTargetBBXPoints3D_pcl);
    
  // Get depth image ideal coordinates
  cv::Mat depthImageIdealCoordinates;
  utl::ocvcalib::getIdealPixelCoordinates(size_depth, K_depth, d_depth, depthImageIdealCoordinates);
  
  //----------------------------------------------------------------------------
  // Calculate depth distortion
  //----------------------------------------------------------------------------

  std::cout << "Calculating depth error images..." << std::endl;
  
  std::vector<cv::Mat> depthImages (depth_IR_frames.size());
  std::vector<cv::Mat> rgbImages (depth_IR_frames.size());
  std::vector<Eigen::Affine3f> targetPoses (depth_IR_frames.size());
  std::vector<cv::Mat> depthMultipliers (depth_IR_frames.size());
  std::vector<std::vector<cv::Point2f> > targetPoints (depth_IR_frames.size());
  std::vector<bool> validFrames (depth_IR_frames.size(), false);
      
  # pragma omp parallel for
  for (size_t frameIdIt = 0; frameIdIt < depth_IR_frames.size(); frameIdIt++)
  {
    int frameId = depth_IR_frames[frameIdIt];
    
    std::cout << frameIdIt << ": " << frameId << std::endl;
        
    // Read images
    cv::Mat imDepth, imIR;
    utl::kinect::readFrame(depth_IR_dirname, frameId, utl::kinect::DEPTH, imDepth);
    utl::kinect::readFrame(depth_IR_dirname, frameId, utl::kinect::IR, imIR);
    imIR.convertTo(imIR, CV_8U, 0.5);
    
    if (imDepth.size() != size_depth)
    {
      std::cout << "Unexpected image size for image " << utl::kinect::generateFilename(frameId, utl::kinect::DEPTH) << std::endl;
      std::cout << "Expected " << size_depth.width << " x " << size_depth.height << ", got " << imDepth.cols << " x " << imDepth.rows << std::endl;
      continue;
    }
    
    if (imIR.size() != size_IR)
    {
      std::cout << "Unexpected image size for image " << utl::kinect::generateFilename(frameId, utl::kinect::IR) << std::endl;
      std::cout << "Expected " << size_IR.width << " x " << size_IR.height << ", got " << imIR.cols << " x " << imIR.rows << std::endl;
      continue;
    }    
    
    std::vector<cv::Point2f> cornersIR, refinedCornersIR;
    if (!utl::ocvcalib::detectTarget (imIR, calibTarget, cornersIR, refinedCornersIR))
    {
      std::cout << "Could not detect corners in '" << utl::kinect::generateFilename(frameId, utl::kinect::RGB) << std::endl;
      continue;
    }
    
    // Get predicted depth
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    utl::kinect::cvDepth2pclCloud_ideal<pcl::PointXYZ>(imDepth, depthImageIdealCoordinates, *cloud);    
    
    // Get calibration target poses
    Eigen::Affine3f targetPose_IR;
    targetPose_IR = utl::ocvcalib::getTargetPose(refinedCornersIR, calibTargetPoints3D, K_IR, d_IR);
    targetPose_IR = cameraPose_IR * targetPose_IR;
        
    // Get the mask of depth pixels around the calibration target
    pcl::PointCloud<pcl::PointXYZ> polygon;
    pcl::transformPointCloud(*calibTargetBBXPoints3D_pcl, polygon, targetPose_IR);
    cv::Mat targetMask = get3DpolygonImageMask(polygon, K_depth, d_depth, size_depth, cameraPose_depth, calibTargetScale);

    
    Eigen::Vector3f planePoint = targetPose_IR.translation();
    Eigen::Vector3f planeNormal = targetPose_IR.rotation().col(2);
    
//     // Fit plane
//     Eigen::Vector4f planeCoefficients;
//     pcl::PointCloud<pcl::PointXYZ> cloud_tmp;
//     std::vector<int> dummy;
//     pcl::removeNaNFromPointCloud<pcl::PointXYZ>(*cloud, cloud_tmp, dummy);
//     utl::cloud::fitPlane<pcl::PointXYZ>(cloud_tmp.makeShared(), planeCoefficients);
//     Eigen::Vector3f planePoint, planeNormal;
//     utl::geom::planeCoefficientsToPointNormal<float>(planeCoefficients, planePoint, planeNormal);
//         
//     targetPose_IR.linear() = utl::geom::alignVectors<float>(Eigen::Vector3f::UnitZ(), planeNormal);
//     targetPose_IR.translation() = cloud->points[0].getVector3fMap();
    
    // Calculate depth multiplier
    cv::Mat depthMultiplier = cv::Mat::zeros(size_depth, CV_32F);
        
    for (size_t x = 0; x < static_cast<size_t>(imDepth.cols); x++)
      for (size_t y = 0; y < static_cast<size_t>(imDepth.rows); y++)
      {
        float d_estimated = cloud->at(x,y).z;
        if (targetMask.at<uchar>(y,x) != 255 || std::isnan(d_estimated))
          continue;
       
        // Get true depth
        float d_true = utl::geom::linePlaneIntersection<float>(Eigen::Vector3f::Zero(), cloud->at(x,y).getVector3fMap(), planePoint, planeNormal)[2];
        float dm = d_true / cloud->at(x,y).z;
        depthMultiplier.at<float>(y,x) = dm;
      }
              
    // Save
    depthImages[frameIdIt] = imDepth;
    rgbImages[frameIdIt] = imIR;
    targetPoses[frameIdIt] = targetPose_IR;
    targetPoints[frameIdIt] = refinedCornersIR;
    depthMultipliers[frameIdIt] = depthMultiplier;
    validFrames[frameIdIt] = true;
  }
  
  // Remove invalid frames
  depthImages       = utl::stdvec::vectorFilter(depthImages, validFrames);
  rgbImages         = utl::stdvec::vectorFilter(rgbImages, validFrames);
  targetPoses       = utl::stdvec::vectorFilter(targetPoses, validFrames);
  targetPoints      = utl::stdvec::vectorFilter(targetPoints, validFrames);
  depthMultipliers  = utl::stdvec::vectorFilter(depthMultipliers, validFrames);
  
  //----------------------------------------------------------------------------
  // Calculate depth multiplier maps
  //----------------------------------------------------------------------------

  std::cout << "Calculating depth distortion parameters..." << std::endl;
  
  std::vector<float> DM_distances = {0.5, 1.0, 1.5, 2.0, 2.5};
//   std::vector<float> DM_distances = {0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5};
  std::vector<cv::Mat> DM_maps (DM_distances.size());
  std::vector<cv::Mat> DM_weights (DM_distances.size());

  for (size_t mapId = 0; mapId < DM_distances.size(); mapId++)
  {
    DM_maps[mapId] = cv::Mat::zeros(size_depth, CV_32F);
    DM_weights[mapId] = cv::Mat::zeros(size_depth, CV_32F);
  }

  for (size_t frameId = 0; frameId < depthImages.size(); frameId++)
  {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//     cloud = clouds[frameIdIt];
    cv::Mat imDepth = depthImages[frameId];
    for (size_t x = 0; x < static_cast<size_t>(imDepth.cols); x++)
      for (size_t y = 0; y < static_cast<size_t>(imDepth.rows); y++)
      {
        float d_estimated = static_cast<float>(imDepth.at<unsigned short>(y,x)) / 1000.0f;
        float dm          = depthMultipliers[frameId].at<float>(y, x);
                
        if (d_estimated == 0 || dm == 0)
          continue;

        // Update map
        std::pair<int,int> mapIndices = utl::stdvec::nearestValues(DM_distances, d_estimated);
        
        if (mapIndices.first != -1)
        {
          DM_maps[mapIndices.first].at<float>(y,x) += dm;
          DM_weights[mapIndices.first].at<float>(y,x) += 1.0;
        }
        
        if (mapIndices.second != -1)
        {
          DM_maps[mapIndices.second].at<float>(y,x) += dm;
          DM_weights[mapIndices.second].at<float>(y,x) += 1.0;
        }
      }
  }
    
  // Normalize depth multiplier maps
  for (size_t dmId = 0; dmId < DM_distances.size(); dmId++)
  {
    // Replace zero weights with wights of one
    cv::Mat invalidMultiplierMask;
    cv::threshold(DM_weights[dmId], invalidMultiplierMask, 0.0, 1, CV_THRESH_BINARY_INV );
    invalidMultiplierMask.convertTo(invalidMultiplierMask, CV_8U, 255);
    cv::Mat defaultWeights = cv::Mat::ones(DM_weights[dmId].size(), CV_32F);
    defaultWeights.copyTo(DM_weights[dmId], invalidMultiplierMask);
    defaultWeights.copyTo(DM_maps[dmId], invalidMultiplierMask);
    
    // Normalize the depth multipliers by the number of observations
    DM_maps[dmId] = DM_maps[dmId] / DM_weights[dmId];
  }
  
  for (size_t dmId = 0; dmId < DM_distances.size(); dmId++)
    cv::imshow("Depth multiplier " + utl::str::to_string(DM_distances[dmId], 1) + " meters", visualizeDepthMultiplierMap(DM_maps[dmId]));
    

  //----------------------------------------------------------------------------
  // Save maps
  //----------------------------------------------------------------------------
  
  std::string depthDistortionFilename = utl::fs::fullfile(calibrationResultDirname, "depth_distortion.xml");
  if (utl::kinect::writeDepthDistortionParameters(depthDistortionFilename, DM_maps, DM_distances))
  {
    std::cout << "Depth distortion parameters saved to '" << depthDistortionFilename << "'" << std::endl;
  }
  else
  {
    std::cout << "Could not save depth distortion parameters saved to '" << depthDistortionFilename << "'" << std::endl;
  }
  
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
      
      // Add camera frustums
      utl::pclvis::showCamera(visualizer, K_depth_eig, size_depth.height, size_depth.width, cameraPose_depth, "depth_cam", utl::pclvis::Color(1.0, 0.0, 0.0));
      utl::pclvis::showCamera(visualizer, K_IR_eig, size_IR.height, size_IR.width, cameraPose_IR, "ir_cam", utl::pclvis::Color(0.0, 0.0, 1.0));
            
      // Get iterator
      visState.iterator_ = utl::math::clampValueCircular<int>(visState.iterator_, 0, depthImages.size()-1);
      
      // Add text
      visualizer.addText("Image " + std::to_string(visState.iterator_+1) + " / " + std::to_string(depthImages.size()), 0, 75, 24, 1.0, 1.0, 1.0);      
      
      // Read images
      cv::Mat imDepth = depthImages[visState.iterator_];
      cv::Mat imIR = rgbImages[visState.iterator_];
      std::vector<cv::Point2f> curTargetPoints = targetPoints[visState.iterator_];

      // Display plane and cloud
      Eigen::Affine3f targetPose_IR = targetPoses[visState.iterator_];
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      
      utl::pclvis::showPlane(visualizer, targetPose_IR, "ir_plane", 0.5, utl::pclvis::Color(0.0, 0.0, 1.0), 0.5);
      
      if (visState.displayState_ == VisState::ORIGINAL_DEPTH)
      {
        
      }
      else if (visState.displayState_ == VisState::UNDISTORTED_DEPTH)
      {
        utl::kinect::correctDepthDistortion(imDepth.clone(), imDepth, DM_maps, DM_distances);
      }

      cv::Mat lol;
      cv::Mat dm_used = utl::kinect::correctDepthDistortion(imDepth.clone(), lol, DM_maps, DM_distances);
      cv::imshow("Depth distortion used", visualizeDepthMultiplierMap(dm_used));
      cv::imshow("True distortion", visualizeDepthMultiplierMap(depthMultipliers[visState.iterator_]));
      
      
      utl::kinect::cvDepth2pclCloud_ideal<pcl::PointXYZ>(imDepth, depthImageIdealCoordinates, *cloud);      
      utl::pclvis::showPointCloud<pcl::PointXYZ>(visualizer, cloud, "cloud_ideal", 1, utl::pclvis::Color(1.0, 0.0, 0.0));
            
      if (visState.showTarget_)
        showTargetPoints(visualizer, calibTargetPoints3D_pcl, targetPose_IR, "ir_target", utl::pclvis::Color(0.0, 1.0, 1.0));

      // Get depth error image
      cv::Mat depthMultiplier = depthMultipliers[visState.iterator_];
      cv::Mat depthMultiplierVis = visualizeDepthMultiplierMap(depthMultipliers[visState.iterator_]);
      
      // Generate target mask
      pcl::PointCloud<pcl::PointXYZ> polygon;
      pcl::transformPointCloud(*calibTargetBBXPoints3D_pcl, polygon, targetPose_IR);
            
      utl::pclvis::showPointCloud<pcl::PointXYZ>(visualizer, polygon.makeShared(), "pol", 10.0, utl::pclvis::Color(0.0, 1.0, 0.0));
      
      cv::Mat targetMaskDepth = get3DpolygonImageMask(polygon, K_depth, d_depth, size_depth, cameraPose_depth, calibTargetScale);
      cv::Mat targetMaskIR    = get3DpolygonImageMask(polygon, K_IR, d_IR, size_IR, cameraPose_IR, calibTargetScale);
      
      cv::Mat imDepthNormalized = utl::ocv::visualizeMatrix(imDepth);
      cv::Mat imDepthVis = imDepthNormalized * 0.5;
      imDepthNormalized.copyTo(imDepthVis, targetMaskDepth);
      cv::Mat imIRVis =  imIR * 10;
      imIR.copyTo(imIRVis, targetMaskIR);
      
      cv::imshow("IR image", imIRVis);
      cv::imshow("Depth image", imDepthVis);
    }
    
    // Spin once
    visualizer.spinOnce();
    boost::this_thread::sleep (boost::posix_time::milliseconds (100));
    cv::waitKey(1);
  }
  
  return 0; 
}