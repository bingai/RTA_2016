// STD includes
#include <iostream>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>

// Utilities
#include <namaris/utilities/filesystem.hpp>

// Project includes
#include "openni_tools/calibration/openni_calibration.h"

////////////////////////////////////////////////////////////////////////////////
void printHelp(char **argv)
{
  std::cout << "Calibrate intrinsics parameters of an OpenNI device." << std::endl;
  std::cout << std::endl;
  std::cout << "Usage: " << utl::fs::getBasename(std::string(argv[0])) << " <input_dir> [parameters]" << std::endl;
  std::cout <<
  "<input_dir> must contain a directory named 'depth_rgb_ir' containing images of a checkerboard taken with IR and color cameras" <<
  std::endl;
  std::cout << std::endl;
  std::cout << "Parameters:" << std::endl;
  std::cout << "      -h:               show this message" << std::endl;
  std::cout << "      -show_corners     show detected corners" << std::endl;
  std::cout << "      -ir               calibrate IR camera" << std::endl;
  std::cout << "      -rgb              calibrate RGB camera" << std::endl;
  std::cout << "      -all              calibrate both cameras camera" << std::endl;
  std::cout << "      -n                number of images that will be used for calibration" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
void parseCommandLine(int argc, char **argv,
                      std::string &input_dir,
                      bool &print_help,
                      bool &show_corners,
                      bool &calibrate_ir,
                      bool &calibrate_rgb,
                      int &num_images_used
)
{
  // Check parameters
  for (size_t i = 1; i < static_cast<size_t>(argc); i++)
  {
    std::string curParameter(argv[i]);

    if (curParameter == "-h")
      print_help = true;

    else if (curParameter == "-show_corners")
      show_corners = true;

    else if (curParameter == "-ir")
      calibrate_ir = true;

    else if (curParameter == "-rgb")
      calibrate_rgb = true;

    else if (curParameter == "-all")
    {
      calibrate_ir = true;
      calibrate_rgb = true;
    }

    else if (curParameter == "-n")
      num_images_used = std::stoi(std::string(argv[++i]));

    else if (curParameter[0] != '-' && input_dir.empty())
      input_dir = curParameter;

    else
      std::cout << "Unknown parameter '" << curParameter << "'" << std::endl;
  }
}

////////////////////////////////////////////////////////////////////////////////
int calibrateIntrinsics(const std::string &image_dirname,
                        const utl::kinect::ImageType image_type,
                        const int num_images_used,
                        const int delay,
                        cv::Mat &K, cv::Mat &d, cv::Size &image_size
)
{
  //----------------------------------------------------------------------------
  // Prepare variables
  //----------------------------------------------------------------------------

  std::vector<std::string> imageFilenames;
  std::vector<int> frameIndices;
  int step;

  cv::Mat image;
  utl::ocvcalib::CalibTarget calibTarget = defaultCalibTarget();
  std::vector<std::vector<cv::Point2f> > cornerPoints(0);

  //----------------------------------------------------------------------------
  // Get available images
  //----------------------------------------------------------------------------

  std::cout << "Reading images from " << image_dirname << std::endl;

  // Get frames
  utl::fs::dir(utl::fs::fullfile(image_dirname, "*.png"), imageFilenames);
  for (size_t imId = 0; imId < imageFilenames.size(); imId++)
  {
    int curFrameIndex;
    utl::kinect::ImageType curImageType;
    if (!utl::kinect::getImageInfo(imageFilenames[imId], curFrameIndex, curImageType))
      continue;

    if (curImageType == image_type)
      frameIndices.push_back(curFrameIndex);
  }

  if (num_images_used == -1)
    step = 1;
  else
    step = frameIndices.size() / num_images_used;

  // Report!
  if (frameIndices.size() > 0)
  {
    std::cout << frameIndices.size() << " frames found." << std::endl;
    std::cout << "Using every " << step << " image." << std::endl;
    std::cout << "Using a total of " << frameIndices.size() / step << " images." << std::endl;
  }
  else
  {
    std::cout << "No IR frames found." << std::endl;
    return -1;
  }

  //----------------------------------------------------------------------------
  // Extract corners
  //----------------------------------------------------------------------------

  std::cout << "Extracting corners..." << std::endl;

  // Read first images to get expected image size
  utl::kinect::readFrame(image_dirname, frameIndices[0], utl::kinect::IR, image);
  image_size = image.size();
  std::cout << "Image size: " << image_size.width << " x " << image_size.height << std::endl;

  // Extract corners
  for (size_t frameIt = 0; frameIt < frameIndices.size(); frameIt += step)
  {
    int frameId = frameIndices[frameIt];
    std::string filename = utl::kinect::generateFilename(frameId, image_type);

    utl::kinect::readFrame(image_dirname, frameId, image_type, image);
    if (image_type == utl::kinect::IR)
      image.convertTo(image, CV_8U, 0.5);
    else if (image_type == utl::kinect::RGB)
      cv::cvtColor(image, image, CV_BGR2GRAY);

    // Check sizes
    if (image.size() != image_size)
    {
      std::cout << "Unexpected image size for image " << filename << "( " << image.cols << " x " << image.rows << ")." << std::endl;
      std::cout << "Skipping this frame." << std::endl;
      continue;
    }

    // Extract corners
    std::vector<cv::Point2f> corners, refinedCorners;
    if (!utl::ocvcalib::detectTarget(image, calibTarget, corners, refinedCorners))
    {
      std::cout << "Could not detect corners in '" << filename << std::endl;
      continue;
    }
    cornerPoints.push_back(refinedCorners);

    // Show detected corners
    cv::Mat cornersDisplay;
    utl::ocvcalib::drawChessboardCorners(image, calibTarget.size_, refinedCorners, cornersDisplay);

    std::string text0 = "Image " + std::to_string(frameId + 1) + " / " + std::to_string(frameIndices.size());
    std::string text1 = filename;
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.7;
    int thickness = 2;
    cv::Scalar fontColor(255, 255, 255);
    cv::Point textOrg(20, cornersDisplay.rows - 150);
    cv::putText(cornersDisplay, text0, textOrg, fontFace, fontScale, fontColor, thickness, 8);
    textOrg.y += 30;
    cv::putText(cornersDisplay, text1, textOrg, fontFace, fontScale, fontColor, thickness, 8);

    cv::imshow("Chessboard corners", cornersDisplay);
    cv::waitKey(delay);
  }

  cvDestroyWindow("Chessboard corners");

  std::cout << "Chessboard pattern detected in " << cornerPoints.size() << " images." << std::endl;

  //----------------------------------------------------------------------------
  // Calibrate
  //----------------------------------------------------------------------------

  std::cout << "Calibrating..." << std::endl;

  int calibrationFlags = 0;
  std::vector<Eigen::Affine3f> targetPoses;
  double totalAvgErr = 0;

  clock_t begin = clock();

  bool ok = utl::ocvcalib::intrinsicCalibration(cornerPoints, image_size,
                                                calibTarget,
                                                K, d,
                                                targetPoses,
                                                totalAvgErr,
                                                calibrationFlags
  );

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

  return 0;
}

int main(int argc, char **argv)
{
  //----------------------------------------------------------------------------
  // Parse command line arguments
  //----------------------------------------------------------------------------

  std::string calibrationImageDirname = "";
  bool print_help = false;
  bool show_corners = false;
  bool calibrateIr = false;
  bool calibrateRgb = false;
  int numImagesUsed = -1;
  int delay;

  parseCommandLine(argc, argv, calibrationImageDirname, print_help, show_corners, calibrateIr, calibrateRgb, numImagesUsed);

  if (print_help)
  {
    printHelp(argv);
    return 0;
  }

  if (!calibrateIr && !calibrateRgb)
  {
    calibrateIr = true;
    calibrateRgb = true;
  }

  // Delay for detected corner display
  show_corners ? delay = 0 : delay = 1;

  //----------------------------------------------------------------------------
  // Check input parameters
  //----------------------------------------------------------------------------

  std::string calibrationResultDirname = utl::fs::fullfile(calibrationImageDirname, "openni_calib");

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


  //----------------------------------------------------------------------------
  // Prepare calibration variables
  //----------------------------------------------------------------------------

  cv::Mat K_IR;
  cv::Mat d_IR;
  cv::Size size_IR;

  cv::Mat K_RGB;
  cv::Mat d_RGB;
  cv::Size size_RGB;


  //----------------------------------------------------------------------------
  // Calibrate
  //----------------------------------------------------------------------------

  if (calibrateIr)
  {
    std::cout << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "Calibrating IR camera" << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    std::cout << std::endl;

    // Calibrate
    int result = calibrateIntrinsics(calibrationImageDirname, utl::kinect::IR, numImagesUsed, delay, K_IR, d_IR, size_IR);

    // Save
    if (result == 0)
    {
      // Create directory
      if (!utl::fs::isDirectory(calibrationResultDirname))
        utl::fs::createDir(calibrationResultDirname);

      // Save calibration
      if (!utl::ocvcalib::writeIntrinsicCalibrationParameters(utl::fs::fullfile(calibrationResultDirname, "ir_calib.xml"),
                                                              K_IR, d_IR, size_IR, "IR")
          )
      {
        return -1;
      }

      std::cout << "Done!" << std::endl;
    }
  }

  if (calibrateRgb)
  {
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "Calibrating RGB camera" << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    std::cout << std::endl;

    // Calibrate
    int result = calibrateIntrinsics(calibrationImageDirname, utl::kinect::RGB, numImagesUsed, delay, K_RGB, d_RGB, size_RGB);

    // Save
    if (result == 0)
    {
      // Create directory
      if (!utl::fs::isDirectory(calibrationResultDirname))
        utl::fs::createDir(calibrationResultDirname);

      // Save calibration
      if (!utl::ocvcalib::writeIntrinsicCalibrationParameters(utl::fs::fullfile(calibrationResultDirname, "rgb_calib.xml"),
                                                              K_RGB, d_RGB, size_RGB, "RGB")
          )
      {
        return -1;
      }

      std::cout << "Done!" << std::endl;
    }
  }

  return 0;
}