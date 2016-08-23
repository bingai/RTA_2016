// STD includes
#include <iostream>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>

// Utilities
#include <namaris/utilities/filesystem.hpp>

// Project includes
#include "openni_tools/calibration/openni_calibration.h"

////////////////////////////////////////////////////////////////////////////////
void printHelp (char** argv)
{
  std::cout << "Calibrate extrinsic parameters of an OpenNI device." << std::endl;
  std::cout << std::endl;
  std::cout << "Usage: " << utl::fs::getBasename(std::string(argv[0])) << " <input_dir> [parameters]" << std::endl;
  std::cout << "<input_dir> must contain a directory named 'depth_rgb_ir' containing images of a checkerboard taken with IR and color cameras" << std::endl;
  std::cout << std::endl;
  std::cout << "Parameters:" << std::endl;
  std::cout << "      -h:               show this message" << std::endl;
  std::cout << "      -show_corners     show detected corners" << std::endl;
  std::cout << "      -fix              fix intrinsic calibration parameters" << std::endl;
  std::cout << "      -n                number of images that will be used for calibration" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
void parseCommandLine ( int argc, char** argv,
                        std::string &input_dir,
                        bool &print_help,
                        bool &show_corners,
                        bool &fix_intrinsics,
                        int &num_images_used
                      )
{   
  // Check parameters
  for (size_t i = 1; i < static_cast<size_t>(argc); i++)
  {
    std::string curParameter (argv[i]);
    
    if (curParameter == "-h")
      print_help = true;
    
    else if (curParameter == "-show_corners")
      show_corners = true;
    
    else if (curParameter == "-fix")
      fix_intrinsics = true;
    
    else if (curParameter == "-n")
      num_images_used = std::stoi(std::string(argv[++i]));
    
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
  
  std::string baseDirname = "";
  bool print_help = false;
  bool show_corners = false;
  bool fixIntrinsics = false;
  int numImagesUsed = -1;
  int delay;
  
  parseCommandLine(argc, argv, baseDirname, print_help, show_corners, fixIntrinsics, numImagesUsed);
  
  if (print_help)
  {
    printHelp(argv);
    return 0;
  }
  
  // Delay for detected corner display
  show_corners ? delay = 0 : delay = 1;  
      
  //----------------------------------------------------------------------------
  // Check input parameters
  //----------------------------------------------------------------------------

  std::string calibrationResultDirname = utl::fs::fullfile(baseDirname, "openni_calib");
  std::string imageDirname = utl::fs::fullfile(baseDirname, "depth_rgb_ir");
    
  if (baseDirname == "")
  {
    std::cout << "You must provide a directory with images used for calibration." << std::endl;
    return -1;
  }
  
  if (!utl::fs::isDirectory(baseDirname))
  {
    std::cout << "Input directory provided not exist or is not a directory (" << baseDirname << ")." << std::endl;
    return -1;
  }

  
  if (!utl::fs::isDirectory(imageDirname))
  {
    std::cout << "Input directory provided does not contain a directory 'depth_rgb_ir' (" << baseDirname << ")." << std::endl;
    return -1;
  }
  
  //----------------------------------------------------------------------------
  // Prepare calibration variables
  //----------------------------------------------------------------------------
  
  cv::Mat K_RGB, K_IR;
  cv::Mat d_RGB, d_IR;
  cv::Size size_RGB, size_IR;
  cv::Mat RGBtoIR_R, RGBtoIR_t;
  utl::ocvcalib::CalibTarget calibTarget = defaultCalibTarget();
    
  //----------------------------------------------------------------------------
  // Read intrinsics if fix intrinsics is passed
  //----------------------------------------------------------------------------
  
  if (fixIntrinsics)
  {
    std::cout << "Looking for precomputed intrinsic parameters... " << std::endl;;
    std::string irIntrinsicsFilename  = utl::fs::fullfile(calibrationResultDirname, "ir_calib.xml");
    std::string rgbIntrinsicsFilename = utl::fs::fullfile(calibrationResultDirname, "rgb_calib.xml");
    
    // Load IR
    if (!utl::fs::isFile(irIntrinsicsFilename))
    {
      std::cout << "Could not load IR intrinsics from file '" << irIntrinsicsFilename << "'. File does not exist." << std::endl;
      return -1;
    }
    
    if (!utl::ocvcalib::readIntrinsicCalibrationParameters ( irIntrinsicsFilename, K_IR, d_IR, size_IR, "IR"))
    {
      return -1;
    }
    std::cout << "  Loaded IR calibration from '" << irIntrinsicsFilename << "'" << std::endl;
    
    // Load RGB
    if (!utl::fs::isFile(rgbIntrinsicsFilename))
    {
      std::cout << "Could not load RGB intrinsics from file '" << rgbIntrinsicsFilename << "'. File does not exist." << std::endl;
      return -1;
    }
    
    if (!utl::ocvcalib::readIntrinsicCalibrationParameters ( rgbIntrinsicsFilename, K_RGB, d_RGB, size_RGB, "RGB"))
    {
      return -1;
    }
    std::cout << "  Loaded RGB calibration from '" << rgbIntrinsicsFilename << "'" << std::endl;
  }
  
  //----------------------------------------------------------------------------
  // Get a list of RGB IR image pairs
  //----------------------------------------------------------------------------

  std::cout << "Reading images from " << imageDirname << std::endl;
  
  // Check that directory with required images exists
  if (!utl::fs::isDirectory(imageDirname))
  {
    std::cout << "Image directory does not exist." << std::endl;
    std::cout << "Could not find '" << imageDirname << "'." << std::endl;
    return -1;
  }
  
  // Get RGB and IR frame indices
  std::vector<std::string> imageFilenames;
  utl::fs::dir(utl::fs::fullfile(imageDirname, "*.png"), imageFilenames);
  
  std::vector<int> rgbFrameIndices;
  std::vector<int> irFrameIndices;
  std::vector<int> frameIndices;
  
  for (size_t imId = 0; imId < imageFilenames.size(); imId++)
  {
    int curFrameIndex;
    utl::kinect::ImageType curImageType;
    if (!utl::kinect::getImageInfo(imageFilenames[imId], curFrameIndex, curImageType))
      continue;
    
    if (curImageType == utl::kinect::ImageType::RGB)
      rgbFrameIndices.push_back(curFrameIndex);
    else if (curImageType == utl::kinect::ImageType::IR)
      irFrameIndices.push_back(curFrameIndex);
  }
  
  // Get RGB IR frame pairs
  for (auto rgbFrameIt = rgbFrameIndices.begin(); rgbFrameIt != rgbFrameIndices.end(); rgbFrameIt++)
  {
    if (std::find(irFrameIndices.begin(), irFrameIndices.end(), *rgbFrameIt) != irFrameIndices.end())
      frameIndices.push_back(*rgbFrameIt);
  }
  
  if (frameIndices.size() > 0)
    std::cout << frameIndices.size() << " IR RGB frame pairs found." << std::endl;
  else
  {
    std::cout << "No IR RGB frame pairs found." << std::endl;
    return -1;
  }
      
  int step = irFrameIndices.size() / numImagesUsed;
  
  std::cout << "Using every " << step << " image." << std::endl;
  
  //----------------------------------------------------------------------------
  // Extract corners
  //----------------------------------------------------------------------------  
  
  std::cout << "Extracting corners..." << std::endl;
    
  cv::Mat imIR, imRGB;
  std::vector<std::vector<cv::Point2f> > cornerPoints_IR  (0);
  std::vector<std::vector<cv::Point2f> > cornerPoints_RGB (0);
  
  // Read first images to get the sizes
  utl::kinect::readFrame(imageDirname, frameIndices[0], utl::kinect::IR, imIR);
  utl::kinect::readFrame(imageDirname, frameIndices[0], utl::kinect::RGB, imRGB);
  size_IR = imIR.size();
  size_RGB = imRGB.size();
  std::cout << "IR image size: " << size_IR.width << " x " << size_IR.height << std::endl;
  std::cout << "RGB image size: " << size_RGB.width << " x " << size_RGB.height << std::endl;
    
  for (size_t frameIt = 0; frameIt < frameIndices.size(); frameIt+=step)
  {
    int frameId = frameIndices[frameIt];
    std::string filenameRGB = utl::kinect::generateFilename(frameId, utl::kinect::RGB);
    std::string filenameIR  = utl::kinect::generateFilename(frameId, utl::kinect::IR);
    
    // Read images
    utl::kinect::readFrame(imageDirname, frameId, utl::kinect::IR, imIR);
    utl::kinect::readFrame(imageDirname, frameId, utl::kinect::RGB, imRGB);
    imIR.convertTo(imIR, CV_8U, 0.5);
    cv::cvtColor(imRGB, imRGB, CV_BGR2GRAY);
        
    // Check sizes
    if (imIR.size() != size_IR)
    {
      std::cout << "Unexpected image size for image " << filenameIR << "( " << imIR.cols << " x " << imIR.rows << ")." << std::endl;
      std::cout << "Skipping this frame pair." << std::endl;
      continue;
    }

    if (imRGB.size() != size_RGB)
    {
      std::cout << "Unexpected image size for image " << filenameRGB << "( " << imRGB.cols << " x " << imRGB.rows << ")." << std::endl;
      std::cout << "Skipping this frame pair." << std::endl;
      continue;
    }    
            
    // Extract corners
    std::vector<cv::Point2f> cornersIR, refinedCornersIR;    
    if (!utl::ocvcalib::detectTarget (imIR, calibTarget, cornersIR, refinedCornersIR))
    {
      std::cout << "Could not detect corners in '" << filenameIR << std::endl;
      continue;
    }   
    
    std::vector<cv::Point2f> cornersRGB, refinedCornersRGB;
    if (!utl::ocvcalib::detectTarget (imRGB, calibTarget, cornersRGB, refinedCornersRGB))
    {
      std::cout << "Could not detect corners in '" << filenameRGB << std::endl;
      continue;
    }
        
    // Only add corners if they were successfully found in both images
    cornerPoints_IR.push_back(refinedCornersIR);
    cornerPoints_RGB.push_back(refinedCornersRGB);
    
    // Show detected corners
    cv::Mat cornersDisplayIR;
    utl::ocvcalib::drawChessboardCorners(imIR, calibTarget.size_, refinedCornersIR, cornersDisplayIR);
    std::string text0 = "Image " + std::to_string(frameId + 1) + " / " + std::to_string(irFrameIndices.size());
    std::string text1 = filenameIR;
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.7;
    int thickness = 2;
    cv::Scalar fontColor (255, 255, 255);
    cv::Point textOrg(20, cornersDisplayIR.rows - 150);
    cv::putText(cornersDisplayIR, text0, textOrg, fontFace, fontScale, fontColor, thickness, 8);
    textOrg.y += 30;
    cv::putText(cornersDisplayIR, text1, textOrg, fontFace, fontScale, fontColor, thickness, 8);    
    cv::imshow("Chessboard corners IR", cornersDisplayIR);
    cv::Mat cornersDisplayRGB;    
    utl::ocvcalib::drawChessboardCorners(imRGB, calibTarget.size_, refinedCornersRGB, cornersDisplayRGB);
    cv::imshow("Chessboard corners RGB", cornersDisplayRGB);

    cv::waitKey(delay);
  }
    
  std::cout << "Chessboard pattern detected in " << cornerPoints_RGB.size() << " image pairs." << std::endl;
  
  //----------------------------------------------------------------------------
  // Calibrate!
  //----------------------------------------------------------------------------
    
  std::cout << "Running stereo calibration..." << std::endl;

  cv::Mat E, F;
  int calibrationFlags = 0;
  if (fixIntrinsics)
    calibrationFlags = CV_CALIB_FIX_INTRINSIC;
  
  double totalAvgErr = 0;

  clock_t begin = clock();
  
  bool ok = stereoCalibration ( cornerPoints_IR, cornerPoints_RGB, size_IR,
                                calibTarget,
                                K_IR, d_IR,
                                K_RGB, d_RGB,
                                RGBtoIR_R, RGBtoIR_t, E, F,
                                totalAvgErr,
                                calibrationFlags  );
    
  clock_t end = clock();
  std::cout << "Optimisation took: " << double(end - begin) / CLOCKS_PER_SEC << " seconds." << std::endl;
  
  // Check if calibration was successfull
  if (!ok)
  {
    std::cout << "Calibration step failed." << std::endl;
    return -1;
  }
  
  // Print calibration results
  std::cout << "Average reprojection error (in pixels): " << totalAvgErr << std::endl;  
  
  //----------------------------------------------------------------------------
  // Save parameters
  //----------------------------------------------------------------------------

  std::cout << "Saving calibration results..." << std::endl;
  
  // Create directory
  if (!utl::fs::isDirectory(calibrationResultDirname))
    utl::fs::createDir(calibrationResultDirname);
  
  // Save calibration
  if (!utl::ocvcalib::writeStereoCalibrationParameters  ( utl::fs::fullfile(calibrationResultDirname, "ir_rgb_calib.xml"),
                                                          K_IR, K_RGB,
                                                          d_IR, d_RGB,
                                                          size_IR, size_RGB,
                                                          RGBtoIR_R, RGBtoIR_t,
                                                          "IR", "RGB" )
     )
  {
    return -1;
  }
    
  std::cout << "Done!" << std::endl;
  return 0;
}