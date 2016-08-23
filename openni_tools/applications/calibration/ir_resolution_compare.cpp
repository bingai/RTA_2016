#include <iostream>

// OpenCV
#include <opencv2/highgui/highgui.hpp>

// Utilities
#include <utilities/filesystem.hpp>
#include <utilities/opencv.hpp>
#include <utilities/math.hpp>

// Project includes
#include "openni_tools.hpp"


int main( int argc, char** argv )
{
  //----------------------------------------------------------------------------
  // Parse command line arguments
  //----------------------------------------------------------------------------
  
  std::string inputDir;
  
  if (argc < 2)
  {
    std::cout << "You must provide a directory with images." << std::endl;
    return -1;
  }
  else
  {
    inputDir = std::string(argv[1]);
  }
  
  inputDir = utl::fs::fullfile(inputDir, "ir_resolution_test");
  
  //----------------------------------------------------------------------------
  // Check command line arguments
  //----------------------------------------------------------------------------
  
  if (!utl::fs::isDirectory(inputDir))
  {
    std::cout << "Input directory provided not exist or is not a directory '" << inputDir << "'" << std::endl;
    return -1;
  }
  
  //----------------------------------------------------------------------------
  // Get IR RGB image pairs
  //----------------------------------------------------------------------------

  std::cout << "Reading images from " << inputDir << std::endl;
  
  // Get frame indices
  std::vector<std::string> imageFilenames;
  utl::fs::dir(utl::fs::fullfile(inputDir, "*.png"), imageFilenames);
  
  std::vector<int> frameIndices;
  
  for (size_t imId = 0; imId < imageFilenames.size(); imId++)
  {
    int curFrameIndex;
    utl::kinect::ImageType curImageType;
    if (!utl::kinect::getImageInfo(imageFilenames[imId], curFrameIndex, curImageType))
      continue;
    
    frameIndices.push_back(curFrameIndex);
  }
  
  utl::stdvec::uniqueVector(frameIndices);
  
  // Check that we have an even number of frames
  if (frameIndices.size() % 2 != 0)
  {
    std::cout << "Expecting an even number of images, got " << frameIndices.size() << " images." << std::endl;
    std::abort();
  }
  
    
  if (frameIndices.size() > 0)
    std::cout << frameIndices.size() / 2 << " frames found." << std::endl;
  else
  {
    std::cout << "No valid frames were found." << std::endl;
    return 0;
  }  
  
  //----------------------------------------------------------------------------
  // Show images
  //----------------------------------------------------------------------------
  
  bool done = false;
  int curFrameIndex = frameIndices[0];
  
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  double dx = 0;
  double dy = 0;
  double step = 0.5;
  
  while (!done)
  {
    // Limit iterator
    curFrameIndex = utl::math::clampValueCircular<int>(curFrameIndex, 0, frameIndices.size() - 1);
    
    // Read images
    cv::Mat imIR1, imIR2;
    utl::kinect::readFrame(inputDir, curFrameIndex, utl::kinect::ImageType::IR, imIR1);
    utl::kinect::readFrame(inputDir, curFrameIndex+1, utl::kinect::ImageType::IR, imIR2);
    
    // Check that images have the reight size
    if (imIR1.size() != cv::Size(640, 480))
    {
      std::cout << "Expected first image to be of resolution 640 x 480, got " << imIR1.cols << " x " << imIR1.rows << "." << std::endl;
      std::abort();
    }
    
    if (imIR2.size() != cv::Size(1280, 1024))
    {
      std::cout << "Expected second image to be of resolution 1280 x 1024, got " << imIR2.cols << " x " << imIR2.rows << "." << std::endl;
      std::abort();
    }    
    
    // Rescale intensity for visualization
    imIR1 = imIR1 * 300;
    imIR2 = imIR2 * 300;

    // Resize 640 image
    cv::resize(imIR1, imIR1, cv::Size(), 2.0, 2.0);
    
    // Crop 1024 image
    int y_excess = imIR2.rows - imIR1.rows;
    imIR2 = cv::Mat(imIR2, cv::Rect(0, y_excess/2 + 1, imIR1.cols, imIR1.rows));
    
    // Shift 1024 image
    cv::Mat K_shifted = K.clone();
    K_shifted.at<double>(0,2) += dx;
    K_shifted.at<double>(1,2) += dy;
    cv::undistort(imIR2.clone(), imIR2, K_shifted, cv::Mat::zeros(1, 5, CV_64F), K);
    
    // Overlay images
    cv::Mat alignmentDisplay;
    std::vector<cv::Mat> channels (3);
    channels[0] = imIR1;
    channels[2] = cv::Mat::zeros(imIR1.size(), CV_16U);
    channels[1] = imIR2;
    cv::merge(channels, alignmentDisplay);
    
    // Add text
    std::string text0 = "Frame " + std::to_string(curFrameIndex / 2 +1) + " / " + std::to_string(frameIndices.size()/2);
    std::string text1 = "dx = " + utl::str::to_string(dx, 1);
    std::string text2 = "dy = " + utl::str::to_string(dy, 1);
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.7;
    int thickness = 2;
    cv::Point textOrg(20, alignmentDisplay.rows - 100);
    cv::putText(alignmentDisplay, text0, textOrg, fontFace, fontScale, cv::Scalar(255, 255, 255), thickness, 8);
    textOrg.y += 30;
    cv::putText(alignmentDisplay, text1, textOrg, fontFace, fontScale, cv::Scalar(255, 255, 255), thickness, 8);
    textOrg.y += 30;
    cv::putText(alignmentDisplay, text2, textOrg, fontFace, fontScale, cv::Scalar(255, 255, 255), thickness, 8);
    
    // Show images
    cv::imshow("IR 1", imIR1);
    cv::imshow("IR 2", imIR2);
    cv::imshow("Alignment", alignmentDisplay);
    
    // Choose what to do next
    char k = cv::waitKey();    
    
    switch (k)
    {
      case 27:
//       case 'q':
//       case 'Q':
        done = true;
        break;
        
      case '>':
      case '.':
        curFrameIndex +=2;
        break;
        
      case '<':
      case ',':
        curFrameIndex -=2;
        break;
        
      case 'a':
      case 'A':
        dx -= step;
        break;

      case 'd':
      case 'D':
        dx += step;
        break;
        
      case 'w':
      case 'W':
        dy += step;
        break;
        
      case 's':
      case 'S':
        dy -= step;
        break;                
        
    }
  }
  
  return 0;
}