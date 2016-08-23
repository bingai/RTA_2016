// STD includes
#include <iostream>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// OpenNI includes
#include <openni2/OpenNI.h>

// Utilities
#include <namaris/utilities/filesystem.hpp>

// Project
#include <openni_tools/openni_tools.hpp>

////////////////////////////////////////////////////////////////////////////////
void printHelp (char** argv)
{
  std::cout << "Display data recorded with an OpenNI device. Can display single frames as well as oni files." << std::endl;
  std::cout << std::endl;
  std::cout << "Usage: " << utl::fs::getBasename(std::string(argv[0])) <<" <input> [parameters]" << std::endl;
  std::cout << "<input> is either a directory containing single frames or a path to an oni file." << std::endl;
  std::cout << std::endl;
  std::cout << "Parameters:" << std::endl;
  std::cout << "    -h            print this message"                                                 << std::endl;
  std::cout << "    -e            if an oni file is provided will extract individual frames from it"  << std::endl;
  std::cout << "    -f            extract images even if output folder already exists"  << std::endl;
  std::cout << "    -novis        if in extraction mode - don't display the images"     << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
void parseCommandLine(int argc, char** argv, std::string &filename, bool &extract_frames, bool &visualize, bool &force_recompute, bool &print_help)
{
  filename = "";
  print_help = false;
  extract_frames = false;
  force_recompute = false;
  visualize = true;
  
  // Check parameters
  for (size_t i = 1; i < static_cast<size_t>(argc); i++)
  {
    std::string curParameter (argv[i]);
    
    if (curParameter == "-h" || curParameter == "-help")
      print_help = true;
    else if (curParameter == "-f")
      force_recompute = true;
    else if (curParameter == "-e")
      extract_frames = true;
    else if (curParameter == "-novis")
      visualize = false;
    else if (curParameter[0] != '-')
      filename = curParameter;
    else 
      std::cout << "Unknown parameter '" << curParameter << "'" << std::endl;
  }
}

////////////////////////////////////////////////////////////////////////////////
int showFrames (const std::string &dirname)
{
  // Get frame indices
  std::vector<std::string> imageFilenames;
  utl::fs::dir(utl::fs::fullfile(dirname, "*.png"), imageFilenames);
  
  std::vector<int> depthFrameIndices;
  std::vector<int> colorFrameIndices;
  std::vector<int> IRFrameIndices;
  
  for (size_t imId = 0; imId < imageFilenames.size(); imId++)
  {
    int curFrameIndex;
    utl::kinect::ImageType curImageType;
    if (!utl::kinect::getImageInfo(imageFilenames[imId], curFrameIndex, curImageType))
      continue;
    
    switch (curImageType)
    {
      case utl::kinect::DEPTH:
        depthFrameIndices.push_back(curFrameIndex);
        break;
        
      case utl::kinect::RGB:
        colorFrameIndices.push_back(curFrameIndex);
        break;

      case utl::kinect::IR:
        IRFrameIndices.push_back(curFrameIndex);
        break;
    }
  }
  
  std::cout << "  " << depthFrameIndices.size() << " depth frames" << std::endl;
  std::cout << "  " << colorFrameIndices.size() << " color frames" << std::endl;
  std::cout << "  " << IRFrameIndices.size() << " IR frames" << std::endl;
  
  // Add all frame indices together
  std::vector<int> frameIndices = depthFrameIndices;
  utl::stdvec::vectorAppend(frameIndices, colorFrameIndices);
  utl::stdvec::vectorAppend(frameIndices, IRFrameIndices);  

  // Check that there are any frames
  utl::stdvec::uniqueVector(frameIndices);
    
  if (frameIndices.size() == 0)
  {
    std::cout << "No valid frames were found." << std::endl;
    return 0;
  }
  
  // Print controls
  std::cout << "Use arrow keys to cycle between images." << std::endl;
  
  //----------------------------------------------------------------------------
  // Show images
  //----------------------------------------------------------------------------
  
  bool done = false;
  int startFrameIndex = frameIndices[0];
  int curFrameIndex = startFrameIndex;
  std::string filename;
  while (!done)
  {
    // Limit iterator    
    if (curFrameIndex < startFrameIndex)
      curFrameIndex = frameIndices.size() - 1;
    else if (curFrameIndex > static_cast<int>(frameIndices.size() - 1))
      curFrameIndex = startFrameIndex;
    
    // Read images
    cv::Mat imRGB, imIR, imDepth;    
    filename = utl::kinect::generateFilename(frameIndices[curFrameIndex], utl::kinect::ImageType::IR);
    if (utl::fs::isFile(utl::fs::fullfile(dirname, filename)))
    {
      imIR = cv::imread(utl::fs::fullfile(dirname, filename), CV_LOAD_IMAGE_ANYDEPTH+CV_LOAD_IMAGE_GRAYSCALE);
      imIR = imIR * 300;
      cv::imshow("IR", imIR);
    }
    else
    {
      cv::destroyWindow("IR");
    }

    std::string filename = utl::kinect::generateFilename(frameIndices[curFrameIndex], utl::kinect::ImageType::RGB);
    if (utl::fs::isFile(utl::fs::fullfile(dirname, filename)))
    {
      imRGB = cv::imread(utl::fs::fullfile(dirname, filename), CV_LOAD_IMAGE_COLOR);
      cv::imshow("RGB", imRGB);
    }
    else
    {
      cv::destroyWindow("RGB");
    }
    
    filename = utl::kinect::generateFilename(frameIndices[curFrameIndex], utl::kinect::ImageType::DEPTH);
    if (utl::fs::isFile(utl::fs::fullfile(dirname, filename)))
    {
      imDepth = cv::imread(utl::fs::fullfile(dirname, filename), CV_LOAD_IMAGE_ANYDEPTH+CV_LOAD_IMAGE_GRAYSCALE);
      imDepth = imDepth / 6000 * std::numeric_limits<unsigned short>::max();      
      cv::imshow("Depth", imDepth);
    }
    else
    {
      cv::destroyWindow("Depth");
    }    
    
    // Choose what to do next
    char k = cv::waitKey();
    
    switch (k)
    {
      case 27:
        done = true;
        break;
        
      case 83:
        curFrameIndex++;
        break;
        
      case 81:
        curFrameIndex--;
        break;
    }
  }
  
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
void openniShutdown ( openni::Device &device,
                      openni::VideoStream &depth_stream, openni::VideoStream &color_stream, openni::VideoStream &IR_stream,
                      openni::VideoStream **streams)
{
  if (depth_stream.isValid())
  {
    depth_stream.stop();
    depth_stream.destroy();
  }
 
  if (color_stream.isValid())
  {
    color_stream.stop();
    color_stream.destroy();
  }
 
  if (IR_stream.isValid())
  {
    IR_stream.stop();
    IR_stream.destroy();
  }

  if (streams)
    delete[] streams;
  
  device.close();
  openni::OpenNI::shutdown();
}

////////////////////////////////////////////////////////////////////////////////
bool checkOpenniStatus  ( const openni::Status &rc)
{
  if (rc != openni::STATUS_OK)
  {
    std::cout << openni::OpenNI::getExtendedError() << std::endl;
    return false;
  }
  
  return true;
}

////////////////////////////////////////////////////////////////////////////////
int showOniFile (const std::string &filename, const bool extract_frames, const bool force_recompute, const bool visualize)
{
  //----------------------------------------------------------------------------
  // Generate required paths
  //----------------------------------------------------------------------------

  std::string sceneBasename   = utl::fs::getBasenameNoExtension(filename);
  std::string frameDirname    = utl::fs::fullfile(utl::fs::getParentDir(filename), sceneBasename + "_frames");
  
  //----------------------------------------------------------------------------
  // Create output directory
  //----------------------------------------------------------------------------

  if (extract_frames)
  {
    std::cout << "Extracting frames to '" << frameDirname << "'." << std::endl;
    if (utl::fs::isDirectory(frameDirname))
    {
      if (!force_recompute)
      {
        std::cout << "Frame directory '" << frameDirname <<"' already exists" << std::endl;
        std::cout << "Already processed!" << std::endl;
        return 0;
      }
      
      std::cout << "Frame directory '" << frameDirname <<"' already exists. Deleting it." << std::endl;
      if (!utl::fs::deleteDir(frameDirname))
      {
        std::cout << "Could not delete '" << frameDirname <<"'." << std::endl;
        return -1;
      }
    }
    
    if (!utl::fs::createDir(frameDirname))
    {
      std::cout << "Could not create frame directory '" << frameDirname << "'." << std::endl;
      return -1;
    }
  }

  //----------------------------------------------------------------------------
  // OpenNI variables
  //----------------------------------------------------------------------------

  openni::Status rc;
  openni::Device device;
  
  openni::VideoStream depthStream_;
  openni::VideoStream colorStream_;
  openni::VideoStream IRStream_;
  openni::VideoStream **streams_ = NULL;
  
  openni::VideoFrameRef depthFrame_;
  openni::VideoFrameRef colorFrame_;
  openni::VideoFrameRef IRFrame_;
    
  openni::VideoMode depthMode_;
  openni::VideoMode colorMode_;
  openni::VideoMode IRMode_;
  
  cv::Mat imDepth, imIR, imColor;
  
  int numDepthFramesAvailable = 0;
  int numColorFramesAvailable = 0;
  int numIRFramesAvailable = 0;
  
  //----------------------------------------------------------------------------
  // Initialize OpenNI
  //----------------------------------------------------------------------------  
  
  // Open OpenNI device
  openni::OpenNI::initialize();
  rc = device.open(filename.c_str());
  if (!checkOpenniStatus(rc))
  {
    openniShutdown(device, depthStream_, colorStream_, IRStream_, streams_);
    return -1;
  }
  
  // Find existing generators
  bool hasDepth = false;
  bool hasIR = false;
  bool hasColor = false;
  
  std::cout << std::endl;
  std::cout << "ONI file details:" << std::endl;
  
  if (device.hasSensor(openni::SENSOR_DEPTH))
  {
    rc = depthStream_.create(device, openni::SENSOR_DEPTH);
    if (!checkOpenniStatus(rc))
    {
      openniShutdown(device, depthStream_, colorStream_, IRStream_, streams_);
      return -1;
    }

    depthMode_ = depthStream_.getVideoMode();
    imDepth = cv::Mat(depthMode_.getResolutionY(), depthMode_.getResolutionX(), CV_16UC1);
    std::cout << "  Depth:  " << depthMode_.getResolutionX() << " x " << depthMode_.getResolutionY() << " @ " << depthMode_.getFps() << "FPS" << std::endl;
    depthStream_.start();
    hasDepth = true;
  }
  
  if (device.hasSensor(openni::SENSOR_COLOR))
  {
    rc = colorStream_.create(device, openni::SENSOR_COLOR);
    if (!checkOpenniStatus(rc))
    {
      openniShutdown(device, depthStream_, colorStream_, IRStream_, streams_);
      return -1;
    }

    colorMode_ = colorStream_.getVideoMode();
    imColor = cv::Mat(colorMode_.getResolutionY(), colorMode_.getResolutionX(), CV_8UC3);
    std::cout << "  Color:  " << colorMode_.getResolutionX() << " x " << colorMode_.getResolutionY() << " @ " << colorMode_.getFps() << "FPS" << std::endl;
    colorStream_.start();
    hasColor = true;
  }  
  
  if (device.hasSensor(openni::SENSOR_IR))
  {
    rc = IRStream_.create(device, openni::SENSOR_IR);
    if (!checkOpenniStatus(rc))
    {
      openniShutdown(device, depthStream_, colorStream_, IRStream_, streams_);
      return -1;
    }

    IRMode_ = IRStream_.getVideoMode();
    std::cout << "  IR:     " << IRMode_.getResolutionX() << " x " << IRMode_.getResolutionY() << " @ " << IRMode_.getFps() << "FPS" << std::endl;
    imIR = cv::Mat(IRMode_.getResolutionY(), IRMode_.getResolutionX(), CV_16UC1);
    IRStream_.start();
    hasIR = true;
  }

  if (!hasDepth && !hasIR && !hasColor)
  {
    std::cout << "  Could not find any image generators. Please check that oni file is valid." << std::endl;
    openniShutdown(device, depthStream_, colorStream_, IRStream_, streams_);
    return -1;
  }
  
  streams_ = new openni::VideoStream*[3];
  streams_[0] = &depthStream_;
  streams_[1] = &colorStream_;  
  streams_[2] = &IRStream_;
    
  if (hasDepth)
  {
    numDepthFramesAvailable = device.getPlaybackControl()->getNumberOfFrames(depthStream_);
    std::cout << "  " << numDepthFramesAvailable << " depth frames" << std::endl;
  }
  if (hasColor)
  {
    numColorFramesAvailable = device.getPlaybackControl()->getNumberOfFrames(colorStream_);
    std::cout << "  " << numColorFramesAvailable << " color frames" << std::endl;
  }
  if (hasIR)
  {
    numIRFramesAvailable = device.getPlaybackControl()->getNumberOfFrames(IRStream_);
    std::cout << "  " << numIRFramesAvailable << " IR frames" << std::endl;
  }

  //----------------------------------------------------------------------------
  // Loop over data
  //----------------------------------------------------------------------------
  
  int frameId = 0;
  device.getPlaybackControl()->setRepeatEnabled(false);
  
  if (extract_frames)
    device.getPlaybackControl()->setSpeed(-1);
  
  int delay;
  extract_frames ? delay = 1 : delay = 0;  
    
  bool quit = false;
  int depthFrameId;
  int colorFrameId;
  int IRFrameId;

  bool depthReadyToWrite  = false;
  bool colorReadyToWrite  = false;
  bool IRReadyToWrite     = false;
  
  int writeFrameId = 0;
  
  while (!quit)
  { 
    int updatedStreamIndex = -1;
    bool depthUpdated = false;
    bool colorUpdated = false;
    bool IRUpdated = false;
        
    rc = openni::STATUS_OK;
    while (rc == openni::STATUS_OK)
    {
      rc = openni::OpenNI::waitForAnyStream(streams_, 3, &updatedStreamIndex, 0);
      if (rc == openni::STATUS_OK)
      {
        switch (updatedStreamIndex)
        {
          case 0:
            depthStream_.readFrame(&depthFrame_);
            depthFrameId = depthFrame_.getFrameIndex();
            depthUpdated = true;
            depthReadyToWrite = true;
            break;
          case 1:
            colorStream_.readFrame(&colorFrame_);
            colorFrameId = colorFrame_.getFrameIndex();
            colorUpdated = true;
            colorReadyToWrite = true;
            break;
          case 2:
            IRStream_.readFrame(&IRFrame_);
            IRFrameId = IRFrame_.getFrameIndex();
            IRUpdated = true;
            IRReadyToWrite = true;
            break;
          default:
            std::cout << "Error on wait." << std::endl;
        }
      }
    }
        
    if (depthUpdated)
    {
      const openni::DepthPixel* depthImagePix = (const openni::DepthPixel*)depthFrame_.getData();
      memcpy(imDepth.data,depthImagePix,imDepth.cols*imDepth.rows*sizeof(openni::DepthPixel));
      if (visualize)
        cv::imshow("Depth", imDepth / 6000 * std::numeric_limits<unsigned short>::max());
    }
        
    if (colorUpdated)
    {
      const openni::RGB888Pixel* colorImagePix = (const openni::RGB888Pixel*)colorFrame_.getData();
      memcpy(imColor.data,colorImagePix,imColor.cols*imColor.rows*3*sizeof(unsigned char));
      cv::cvtColor(imColor, imColor, CV_RGB2BGR);
      if (visualize)      
        cv::imshow("RGB", imColor);
    }
        
    if (IRUpdated)
    { 
      const openni::Grayscale16Pixel* irImagePix = (const openni::Grayscale16Pixel*)IRFrame_.getData();
      memcpy(imIR.data, irImagePix, imIR.cols*imIR.rows * sizeof(openni::Grayscale16Pixel));
      if (visualize)
        imshow("IR", imIR*300);
    }
    
    // Write frames
    if (extract_frames)
    {
      // If both available streams were updated write
      if (  (!hasDepth || depthReadyToWrite) && 
            (!hasColor || colorReadyToWrite) &&
            (!hasIR    || IRReadyToWrite)  )
      {
        if (hasDepth)
          utl::kinect::writeFrame(frameDirname, writeFrameId, utl::kinect::DEPTH, imDepth);
        if (hasColor)
          utl::kinect::writeFrame(frameDirname, writeFrameId, utl::kinect::RGB, imColor);
        if (hasIR)
          utl::kinect::writeFrame(frameDirname, writeFrameId, utl::kinect::IR, imIR);
        
        writeFrameId++;
        depthReadyToWrite = false;
        colorReadyToWrite = false;
        IRReadyToWrite    = false;
      }
    }
    
    // Check keyboard input
    if (depthUpdated || colorUpdated || IRUpdated)
    {
      char k = cv::waitKey(delay);
      quit = ((k == 27) || (k == 'q') || (k == 'Q'));
    }
    
    // Check if we have recorded all of the frames
    bool allFramesRead = true;
    if (hasDepth) allFramesRead = allFramesRead && depthFrameId == numDepthFramesAvailable;
    if (hasColor) allFramesRead = allFramesRead && colorFrameId == numColorFramesAvailable;
    if (hasIR)    allFramesRead = allFramesRead && IRFrameId    == numIRFramesAvailable;

    quit = quit || allFramesRead;    
  }

  if (extract_frames)
    std::cout << writeFrameId << " frames were extracted." << std::endl;
  
  //----------------------------------------------------------------------------
  // Cleanup
  //----------------------------------------------------------------------------

  openniShutdown(device, depthStream_, colorStream_, IRStream_, streams_);
  
  return 0;
}


////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{  
  //----------------------------------------------------------------------------
  // Parse command line arguments
  //----------------------------------------------------------------------------
  
  std::string inputPath;
  bool visualize;
  bool print_help;
  bool extractFrames;
  bool forceRecompute;
  
  parseCommandLine(argc, argv, inputPath, extractFrames, visualize, forceRecompute, print_help);
  
  if (print_help)
    printHelp(argv);
  
  //----------------------------------------------------------------------------
  // Check command line parameters
  //----------------------------------------------------------------------------

  if (utl::fs::isDirectory(inputPath))
  {
    std::cout << "Input path is a directory. Looking for individual frames..." << std::endl;
    return showFrames(inputPath);
  }
  else if (utl::fs::isFile(inputPath))
  {
    std::cout << "Input path is a file, trying to read an oni file..." << std::endl;
    return showOniFile(inputPath, extractFrames, forceRecompute, visualize);
  }
  else
  {
    std::cout << "Input path does not exist. Please choose a different path." << std::endl;
    return -1;
  }
}
