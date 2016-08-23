#include <openni_tools/capture/openni_capture.h>

// STD includes
#include <iostream>
#include <iomanip>
#include <chrono>
#include <cctype>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// OpenNI2
#include <openni2/PS1080.h>
#include <openni2/PSLink.h>

// Utilities
#include <namaris/utilities/filesystem.hpp>
#include <namaris/utilities/opencv.hpp>

////////////////////////////////////////////////////////////////////////////////
OpenNICapture::OpenNICapture (  const OpenNICapture::CAPTURE_MODE& capture_mode,
                                const std::string& capture_dirname,
                                const std::string &capture_filename_prefix,
                                const bool view_only,
                                const bool verbose  )
  : captureMode_(capture_mode)
  , captureDirname_(capture_dirname)
  , captureFilenamePrefix_(capture_filename_prefix)
  , viewOnly_(view_only)
  , displayImageWidth_ (800)
  , colorStreamRunning_(false)
  , depthStreamRunning_(false)
  , IRStreamRunning_(false)
  , depthLossyCompression_(false)
  , colorLossyCompression_(true)
  , IRLossyCompression_(false)
  , depthColorFrameSync_(true)
  , highResStreams_(false)
  , depthCloseRangeMode_ (false)
  , irEmitterStatus_(true)
  , depthToColorRegistration_(false)
  , depthSmoothing_(false)
  , depthSmoothingNumFrames_(15)
  , showCaptureInfo_(false)
  , capturingOniFile_(false)
  , verbose_(verbose)
{  
  // Initialize stream modes
  defineStreamModes();
  
  // Initialize capture
  if (!initStreams())
    std::abort();
//   listAvailableModes();  
  
  // Set capture modes
  setStreamModes();
  
  // Start streams
  updateStreams();

  // Find oni file maximum index
  std::vector<std::string> oniFilenames;
  utl::fs::dir(utl::fs::fullfile(captureDirname_, captureFilenamePrefix_ + "_*.oni"), oniFilenames);
  curOniIndex_ = -1;
  for (size_t fileIt = 0; fileIt < oniFilenames.size(); fileIt++)
  {
    int id = getOniFilenameIndex(oniFilenames[fileIt]);
    if (id >= 0)
      curOniIndex_ = std::max(curOniIndex_, id);    
  }
  curOniIndex_++;
  
  // Find frame file maximum index
  std::vector<std::string> frameFilenames;
  utl::fs::dir(utl::fs::fullfile(captureDirname_, captureFilenamePrefix_ + "_*.png"), frameFilenames);
  curFrameIndex_ = -1;
  for (size_t fileIt = 0; fileIt < frameFilenames.size(); fileIt++)
  {
    int frameIndex;
    utl::kinect::ImageType curImageType;
    
    if (!utl::kinect::getImageInfo(frameFilenames[fileIt], frameIndex, curImageType, captureFilenamePrefix_))
      continue;
    
    curFrameIndex_ = std::max(curFrameIndex_, frameIndex);
  }
  curFrameIndex_++;
  
  // Display usage
  if (viewOnly_)
    std::cout << "Capture is disabled" << std::endl;
  else
  {
    if (verbose_)
    {
      std::cout << "Images will be written to '" << captureDirname_ << "'" << std::endl;
      std::cout << "Starting frame numbering from " << curFrameIndex_ << std::endl;
      std::cout << "Starting oni file numbering from " << curOniIndex_ << std::endl;
    }
  }

  // Display usage information
  std::cout << "|----------------------------------------------------|" << std::endl;
  std::cout << "|--------------- Change capture mode ----------------|" << std::endl;
  std::cout << "|----------------------------------------------------|" << std::endl;  
  std::cout << "| 1        |  DEPTH                                  |" << std::endl;
  std::cout << "| 2        |  RGB                                    |" << std::endl;
  std::cout << "| 3        |  IR                                     |" << std::endl;
  std::cout << "| 4        |  DEPTH+RGB                              |" << std::endl;
  std::cout << "| 5        |  DEPTH+IR                               |" << std::endl;
  std::cout << "| 6        |  RGB+IR                                 |" << std::endl;
  std::cout << "| 7        |  DEPTH+RGB+IR                           |" << std::endl;
  std::cout << "| N        |  Show capture information               |" << std::endl;
  std::cout << "| R        |  Toggle depth to color registration     |" << std::endl;
  std::cout << "| C        |  Toggle close range depth               |" << std::endl;
  std::cout << "| I        |  Turn IR emitter ON/OFF                 |" << std::endl;
  std::cout << "| S        |  Toggle depth temporal smoothing        |" << std::endl;
  std::cout << "| tab      |  Toggle high resolution RGB and IR      |" << std::endl;

  if (!view_only)
  {
    std::cout << "|----------------------------------------------------|" << std::endl;
    std::cout << "|---------------- Capture controls ------------------|" << std::endl;
    std::cout << "|----------------------------------------------------|" << std::endl;
    std::cout << "| Space    |  Capture single frame                   |" << std::endl;
    std::cout << "| D        |  Start recording oni file with a delay  |" << std::endl;
    std::cout << "| X        |  Stop recording oni file                |" << std::endl;
  }

  std::cout << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
OpenNICapture::~OpenNICapture ()
{
  if (recorder_.isValid())
    stopOniCapture();
  
  if (colorStream_.isValid())
  {
    if (colorStreamRunning_)
      colorStream_.stop();
    colorStream_.destroy();
  }

  if (depthStream_.isValid())
  {
    if (depthStreamRunning_)
      depthStream_.stop();
    depthStream_.destroy();
  }
  
  if (IRStream_.isValid())
  {
    if (IRStreamRunning_)
      IRStream_.stop();
    IRStream_.destroy();
  }
  device_.close();
  
  delete[] streams_;  
  
  openni::OpenNI::shutdown();
}

////////////////////////////////////////////////////////////////////////////////
void OpenNICapture::defineStreamModes()
{
  // Color
  colorModeHighRes_.setResolution(1280, 1024);
  colorModeHighRes_.setFps(30);
  colorModeHighRes_.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
  
  colorModeLowRes_.setResolution(640, 480);
  colorModeLowRes_.setFps(30);
  colorModeLowRes_.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
  
  // Depth
  depthMode_.setResolution(640, 480);
  depthMode_.setFps(30);
  depthMode_.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
  
  // IR
  IRModeHighRes_.setResolution(1280, 1024);
  IRModeHighRes_.setFps(30);
  IRModeHighRes_.setPixelFormat(openni::PIXEL_FORMAT_GRAY16);
  
  IRModeLowRes_.setResolution(640, 480);
  IRModeLowRes_.setFps(30);
  IRModeLowRes_.setPixelFormat(openni::PIXEL_FORMAT_GRAY16);
}

////////////////////////////////////////////////////////////////////////////////
void OpenNICapture::setStreamModes()
{
  openni::Status rc;
  
  //----------------------------------------------------------------------------
  // Color
  if (colorStreamRunning_)
  {
    colorStream_.stop();
    colorStreamRunning_ = false;
  }
  
  if (highResStreams_)
    colorMode_ = colorModeHighRes_;
  else
    colorMode_ = colorModeLowRes_;
  
  rc = colorStream_.setVideoMode(colorMode_);
  if (rc != openni::STATUS_OK)
  {
    std::cout << "[OpenNICapture::setStreamModes] Failed to set color mode." << std::endl;
    std::abort();
  }
  else
    colorStream_.setMirroringEnabled(false);
  
  colorImage_         = cv::Mat(cv::Size(colorMode_.getResolutionX(), colorMode_.getResolutionY()), CV_8UC3);
  colorImageDisplay_  = cv::Mat(cv::Size(colorMode_.getResolutionX(), colorMode_.getResolutionY()), CV_8UC3);
  
  //----------------------------------------------------------------------------
  // Depth
  if (depthStreamRunning_)
  {
    depthStream_.stop();
    depthStreamRunning_ = false;
  }
    
  rc = depthStream_.setVideoMode(depthMode_);
  if (rc != openni::STATUS_OK)
  {
    std::cout << "[OpenNICapture::setStreamModes] Failed to set depth mode." << std::endl;
    std::abort();
  }
  else
    depthStream_.setMirroringEnabled(false);
  
  depthImage_         = cv::Mat(cv::Size(depthMode_.getResolutionX(), depthMode_.getResolutionY()), CV_16UC1);
  depthImageDisplay_  = cv::Mat(cv::Size(depthMode_.getResolutionX(), depthMode_.getResolutionY()), CV_16UC1);
  depthImageSave_     = cv::Mat(cv::Size(depthMode_.getResolutionX(), depthMode_.getResolutionY()), CV_16UC1);
  
  //----------------------------------------------------------------------------
  // IR
  if (IRStreamRunning_)
  {
    IRStream_.stop();
    IRStreamRunning_ = false;
  }
  
  if (highResStreams_)
    IRMode_ = IRModeHighRes_;
  else
    IRMode_ = IRModeLowRes_;
  
  rc = IRStream_.setVideoMode(IRMode_);
  if (rc != openni::STATUS_OK)
  {
    std::cout << "[OpenNICapture::setStreamModes] Failed to set IR mode." << std::endl;
    std::abort();
  }
  else
    IRStream_.setMirroringEnabled(false);
  
  IRImage_        = cv::Mat(cv::Size(IRMode_.getResolutionX(), IRMode_.getResolutionY()), CV_16UC1);
  IRImageDisplay_ = cv::Mat(cv::Size(IRMode_.getResolutionX(), IRMode_.getResolutionY()), CV_16UC1);
}

////////////////////////////////////////////////////////////////////////////////
bool OpenNICapture::initStreams()
{
  openni::Status rc = openni::STATUS_OK;
  
  //----------------------------------------------------------------------------
  // Initialize OpenNI
  rc = openni::OpenNI::initialize();
  std::cout << "Initialization ... ";
  if (rc != openni::STATUS_OK)
  {
    std::cout << std::endl;
    std::cout << openni::OpenNI::getExtendedError() << std::endl;
  }
  else
  {
    std::cout << "Good!" << std::endl;
  }
  
  std::cout << "Opening device ... ";
  rc = device_.open(openni::ANY_DEVICE);
  if (rc != openni::STATUS_OK)
  {
    std::cout << "Could not open any OpenNI device!" << std::endl;
    std::cout <<  openni::OpenNI::getExtendedError() << std::endl;
    openni::OpenNI::shutdown();
    return false;
  }
  else
  {
    std::cout << "Good!" << std::endl;
  }  
  
  //----------------------------------------------------------------------------
  // Initialize RGB stream
  
  rc = colorStream_.create(device_,openni::SENSOR_COLOR);
  if (rc != openni::STATUS_OK)
  {
    std::cout << "OpenNI: Couldn't find color stream:" << std::endl;
    std::cout << openni::OpenNI::getExtendedError() << std::endl;
    std::abort();
  }
    
  //----------------------------------------------------------------------------
  // Initialize depth stream

  rc = depthStream_.create(device_,openni::SENSOR_DEPTH);
  if (rc != openni::STATUS_OK)
  {
    std::cout << "OpenNI: Couldn't find depth stream:" << std::endl;
    std::cout << openni::OpenNI::getExtendedError() << std::endl;
    std::abort();
  }  
  
  //----------------------------------------------------------------------------
  // Initialize IR stream
  
  rc = IRStream_.create(device_,openni::SENSOR_IR);
  if (rc != openni::STATUS_OK)
  {
    std::cout << "OpenNI: Couldn't find IR stream:" << std::endl;
    std::cout << openni::OpenNI::getExtendedError() << std::endl;
    std::abort();
  }
  
//   openni::Status
  
  streams_ = new openni::VideoStream*[3];
  streams_[0] = &depthStream_;
  streams_[1] = &colorStream_;
  streams_[2] = &IRStream_;
  
  return true;
}

////////////////////////////////////////////////////////////////////////////////
void OpenNICapture::startStream(const openni::SensorType sensor_type)
{
  openni::Status rc = openni::STATUS_OK;
    
  if (sensor_type == openni::SENSOR_DEPTH)
  {    
    if (depthStreamRunning_)
      return;
    
    rc = depthStream_.start();
    if (rc != openni::STATUS_OK)
    {
      std::cout << "[OpenNICapture::startStream] couldn't start depth stream:" << std::endl;
      std::cout << openni::OpenNI::getExtendedError() << std::endl;
      std::abort();
    }
    else
    {
      if (verbose_)
        std::cout << "[OpenNICapture::startStream] Depth stream started." << std::endl;
      depthStreamRunning_ = true;
    }
  }
  
  else if (sensor_type == openni::SENSOR_COLOR)
  {
    if (colorStreamRunning_)
      return;
    
    if (IRStreamRunning_)
      stopStream(openni::SENSOR_IR);
    
    rc = colorStream_.start();
    if (rc != openni::STATUS_OK)
    {
      std::cout << "[OpenNICapture::startStream] Couldn't start color stream:" << std::endl;
      std::cout << openni::OpenNI::getExtendedError() << std::endl;
      std::abort();
    }
    else
    {
      if (verbose_)
        std::cout << "[OpenNICapture::startStream] Color stream started." << std::endl;
      colorStreamRunning_ = true;
    }
  }
  
  else if (sensor_type == openni::SENSOR_IR)
  {    
    if (IRStreamRunning_)
      return;
        
    if (colorStreamRunning_)
      stopStream(openni::SENSOR_COLOR);
    
    rc = IRStream_.start();
    if (rc != openni::STATUS_OK)
    {
      std::cout << "[OpenNICapture::startStream] Couldn't start IR stream:" << std::endl;
      std::cout << openni::OpenNI::getExtendedError() << std::endl;
      std::abort();
    }
    else
    {
      if (verbose_)
        std::cout << "[OpenNICapture::startStream] IR stream started." << std::endl;
      IRStreamRunning_ = true;
    }    
  }
  
  else
  {
    std::cout << "[OpenNICapture::startStream] Unknown stream type." << std::endl;
  }  
}

////////////////////////////////////////////////////////////////////////////////
void OpenNICapture::stopStream(const openni::SensorType sensor_type)
{  
  if (sensor_type == openni::SENSOR_DEPTH && depthStreamRunning_)
  {    
    depthStream_.stop();
    depthStreamRunning_ = false;
    if (verbose_)
      std::cout << "[OpenNICapture::stopStream] depth stream stopped." << std::endl;
  }
 
  else if (sensor_type == openni::SENSOR_COLOR && colorStreamRunning_)
  {
    colorStream_.stop();
    colorStreamRunning_ = false;
    if (verbose_)
      std::cout << "[OpenNICapture::stopStream] color stream stopped." << std::endl;
  }
 
  else if (sensor_type == openni::SENSOR_IR && IRStreamRunning_)
  {    
    IRStream_.stop();
    IRStreamRunning_ = false;
    if (verbose_)
      std::cout << "[OpenNICapture::stopStream] IR stream stopped." << std::endl;    
  }
  
  else
  {
    std::cout << "[OpenNICapture::stopStream] Unknown stream type." << std::endl;
  }
}

////////////////////////////////////////////////////////////////////////////////
void OpenNICapture::updateStreams()
{
  if (captureMode_ == DEPTH || captureMode_ == IR || captureMode_ == DEPTH_RGB || captureMode_ == DEPTH_IR || captureMode_ == DEPTH_RGB_IR)
    startStream(openni::SENSOR_DEPTH);
  
  if (captureMode_ == RGB || captureMode_ == DEPTH_RGB || captureMode_ == DEPTH_RGB_IR)
    startStream(openni::SENSOR_COLOR);
  
  if (captureMode_ == IR || captureMode_ == RGB_IR || captureMode_ == DEPTH_IR)
    startStream(openni::SENSOR_IR);  
  
  // Update capture parameters
  int updatedStreamIndex;
  openni::Status rc = openni::OpenNI::waitForAnyStream(streams_, 3, &updatedStreamIndex, 1000);
  
  if (depthStreamRunning_)
  {
    setIREmitterStatus(irEmitterStatus_);
    setDepthToColorRegistration(depthToColorRegistration_);
  }
  if (depthStreamRunning_ || IRStreamRunning_)
    setDepthCloseRangeMode(depthCloseRangeMode_);
  
  if (depthStreamRunning_ && colorStreamRunning_)
    setDepthColorFrameSync(depthColorFrameSync_);
}

////////////////////////////////////////////////////////////////////////////////
bool OpenNICapture::getDepthToColorRegistration()
{
  if (device_.getImageRegistrationMode() == openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR)
    return true;
  else if (device_.getImageRegistrationMode() == openni::IMAGE_REGISTRATION_OFF)
    return false;
  else
  {
    std::cout << "[OpenNICapture::getDepthToColorRegistration] could not get depth to color registration. Aborting..."  << std::endl;
    std::cout << openni::OpenNI::getExtendedError() << std::endl;    
    std::abort();
  }
}

////////////////////////////////////////////////////////////////////////////////
void OpenNICapture::setDepthToColorRegistration(const bool new_value)
{
  if (getDepthToColorRegistration() == new_value)
    return;
  
  openni::Status rc;
  if (new_value)
    rc = device_.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
  else
    rc = device_.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
  
  if (rc != openni::STATUS_OK)
  {
    std::cout << "Could not turn depth to color registration ON/OFF" << std::endl;
    if (verbose_)
      std::cout << openni::OpenNI::getExtendedError() << std::endl;
  }
  depthToColorRegistration_ = getDepthToColorRegistration();
}

////////////////////////////////////////////////////////////////////////////////
bool OpenNICapture::getDepthCloseRangeMode()
{
  bool value;
  openni::Status rc = depthStream_.getProperty<bool>(XN_STREAM_PROPERTY_CLOSE_RANGE, &value);
  if (rc != openni::STATUS_OK)
  {
    std::cout << "[OpenNICapture::getDepthCloseRangeMode]: could not get depth close range mode. Aborting..." << std::endl;
    std::cout << openni::OpenNI::getExtendedError() << std::endl;
    std::abort();
  }
  return value;
}

////////////////////////////////////////////////////////////////////////////////
void OpenNICapture::setDepthCloseRangeMode(const bool new_value)
{
  if (getDepthCloseRangeMode() == new_value)
    return;
  
  openni::Status rc = depthStream_.setProperty<bool>(XN_STREAM_PROPERTY_CLOSE_RANGE, new_value);
  if (rc != openni::STATUS_OK)
  {
    std::cout << "Could not change depth stream close range mode." << std::endl;
    if (verbose_)
      std::cout << openni::OpenNI::getExtendedError() << std::endl;
  }
  
  depthCloseRangeMode_ = getDepthCloseRangeMode();
}

////////////////////////////////////////////////////////////////////////////////
bool OpenNICapture::getIREmitterStatus()
{
  // NOTE: for some reason device_.getProperty<bool>(XN_MODULE_PROPERTY_EMITTER_STATUS, &value) returns BAD_PARAMETER. So hacking arround that issue.
//   bool value;
//   openni::Status rc = device_.getProperty<bool>(XN_MODULE_PROPERTY_EMITTER_STATE, &value);
//   if (rc != openni::STATUS_OK)
//   {
//     std::cout << "[OpenNICapture::getIREmitterStatus]: could not get IR emitter status. Using prestored value..." << std::endl;
//     std::cout << openni::OpenNI::getExtendedError() << std::endl;
//     std::abort();
//   }
//   return value;
  
  return irEmitterStatus_;
}

////////////////////////////////////////////////////////////////////////////////
void OpenNICapture::setIREmitterStatus(const bool new_value)
{
  if (getIREmitterStatus() == new_value)
    return;
  
  openni::Status rc = device_.setProperty<bool>(XN_MODULE_PROPERTY_EMITTER_STATE, new_value);
  if (rc != openni::STATUS_OK)
  {
    std::cout << "Could not turn IR projector ON/OFF." << std::endl;
    if (verbose_)
      std::cout << openni::OpenNI::getExtendedError() << std::endl;
  }
  else
  {
    irEmitterStatus_ = !getIREmitterStatus();
  }
}

////////////////////////////////////////////////////////////////////////////////
bool OpenNICapture::getDepthColorFrameSync()
{
  return device_.getDepthColorSyncEnabled();
}

////////////////////////////////////////////////////////////////////////////////
void OpenNICapture::setDepthColorFrameSync(const bool new_value)
{
  if (getDepthColorFrameSync() == new_value)
    return;
  
  openni::Status rc = device_.setDepthColorSyncEnabled(new_value);
  
  if (rc != openni::STATUS_OK)
  {
    std::cout << "Could not turn depth color frame sync ON/OFF." << std::endl;
    if (verbose_)
      std::cout << openni::OpenNI::getExtendedError() << std::endl;
  }
  
  depthColorFrameSync_ = getDepthColorFrameSync();
}

////////////////////////////////////////////////////////////////////////////////
void OpenNICapture::listAvailableModes  ( const openni::SensorInfo *sensorInfo  )
{
  // Go through sensor modes
  const openni::Array<openni::VideoMode> &modes = sensorInfo->getSupportedVideoModes();
  for (size_t modeId = 0; modeId < static_cast<size_t>(modes.getSize()); modeId++)
  {
    const openni::VideoMode &curMode = modes[modeId];
    std::cout << "Type:         ";
    
    if (sensorInfo->getSensorType() == openni::SENSOR_COLOR)
      std::cout << "RGB" << std::endl;
    else if (sensorInfo->getSensorType() == openni::SENSOR_DEPTH)
    {
      if (curMode.getPixelFormat() == openni::PIXEL_FORMAT_SHIFT_9_2 || curMode.getPixelFormat() == openni::PIXEL_FORMAT_SHIFT_9_3)
        std::cout << "disparity" << std::endl;
      else
        std::cout << "depth" << std::endl;
    }
    else if (sensorInfo->getSensorType() == openni::SENSOR_IR)
      std::cout << "IR" << std::endl;
    
    std::cout << "Resolution:   " << curMode.getResolutionX() << " x " << curMode.getResolutionY() << std::endl;
    std::cout << "Frame rate:   " << curMode.getFps() << "fps" << std::endl;
    std::cout << "Pixel format: ";
    switch (curMode.getPixelFormat())
    {
      case openni::PIXEL_FORMAT_DEPTH_1_MM:
        std::cout << "PIXEL_FORMAT_DEPTH_1_MM";
        break;
        
      case openni::PIXEL_FORMAT_DEPTH_100_UM:
        std::cout << "PIXEL_FORMAT_DEPTH_100_UM";
        break;

      case openni::PIXEL_FORMAT_SHIFT_9_2:
        std::cout << "PIXEL_FORMAT_SHIFT_9_2";
        break;

      case openni::PIXEL_FORMAT_SHIFT_9_3:
        std::cout << "PIXEL_FORMAT_SHIFT_9_3";
        break;

      case openni::PIXEL_FORMAT_RGB888:
        std::cout << "PIXEL_FORMAT_RGB888";
        break;

      case openni::PIXEL_FORMAT_YUV422:
        std::cout << "PIXEL_FORMAT_YUV422";
        break;

      case openni::PIXEL_FORMAT_GRAY8:
        std::cout << "PIXEL_FORMAT_GRAY8";
        break;

      case openni::PIXEL_FORMAT_GRAY16:
        std::cout << "PIXEL_FORMAT_GRAY16";
        break;

      case openni::PIXEL_FORMAT_JPEG:
        std::cout << "PIXEL_FORMAT_JPEG";
        break;

      case openni::PIXEL_FORMAT_YUYV:
        std::cout << "PIXEL_FORMAT_YUYV";
        break;
        
      default:
        std::cout << "unknown";
        
    }
    std::cout << std::endl << std::endl;
  }  
}

////////////////////////////////////////////////////////////////////////////////
void OpenNICapture::listAvailableModes()
{
  // Check that device is connected
  if (!device_.isValid())
  {
    std::cout << "[OpenNICapture::listAvailableModes] there are no connected devices." << std::endl;
    return;
  }

  // List availabe modes for differnt sensor types
  listAvailableModes(device_.getSensorInfo(openni::SENSOR_DEPTH));
  listAvailableModes(device_.getSensorInfo(openni::SENSOR_COLOR));
  listAvailableModes(device_.getSensorInfo(openni::SENSOR_IR));
}

////////////////////////////////////////////////////////////////////////////////
void OpenNICapture::run()
{
  bool quit = false;

  while (!quit)
  {    
    char k  = retrieveVisualize(captureMode_);
    switch (k)
    {
      // Quit
      case 27:
      case 'q':
      case 'Q':
      {
        quit = true;
        break;
      }

      // Toggle close range mode
      case 'c':
      case 'C':
      {
        setDepthCloseRangeMode(!getDepthCloseRangeMode());
        break;
      }

      // Toggle IR projector on/off
      case 'i':
      case 'I':
      {
        setIREmitterStatus(!getIREmitterStatus());
        break;
      }
      
      case 'r':
      case 'R':
      {
        setDepthToColorRegistration(!depthToColorRegistration_);
        break;
      }
      
      case 'n':
      case 'N':
      {
        showCaptureInfo_ = !showCaptureInfo_;
        break;
      }

      case 's':
      case 'S':
      {
        depthSmoothing_ = !depthSmoothing_;
        break;
      }

      // Number of frames for smoothing
      case '>':
      case '.':
      {
        depthSmoothingNumFrames_++;
        break;
      }

      case '<':
      case ',':
      {
        depthSmoothingNumFrames_ = std::max(1, depthSmoothingNumFrames_-1);
        break;
      }
      
      // Switch resolution
      case 9:
      {
        highResStreams_ = !highResStreams_;
        setStreamModes();
        updateStreams();
        
        break;
      }
      
      // Capture images
      case 32:
      {
        if (!viewOnly_)
          quit = !captureImages();
        else
          std::cout << "Cannot capture images sicne running in view only mode." << std::endl;
        break;
      }      
      
      // Capture oni file with delay
      case 'd':
      case 'D':
      {
        if (!viewOnly_)
          startOniCapture(3);
        else
          std::cout << "Cannot capture images sicne running in view only mode." << std::endl;        
        break;
      }

      // Stop oni capture
      case 'x':
      case 'X':
      {
        if (!viewOnly_)
          stopOniCapture();
        break;
      }      
      
      // Switch capture mode
      case CAPTURE_MODE::DEPTH:
      case CAPTURE_MODE::RGB:        
      case CAPTURE_MODE::IR:
      case CAPTURE_MODE::DEPTH_RGB:
      case CAPTURE_MODE::DEPTH_IR:
      case CAPTURE_MODE::RGB_IR:
      case CAPTURE_MODE::DEPTH_RGB_IR:
      {
        if (captureMode_ != k)
        {
          captureMode_ = (static_cast<CAPTURE_MODE>(k));
          updateStreams();
        }
      }
      
      // Default
      default:
      {
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
bool OpenNICapture::retrieveImage(const openni::SensorType sensor_type)
{
  switch (sensor_type)
  {
    case openni::SENSOR_COLOR:
    {
      colorStream_.readFrame(&colorFrame_);
      if (colorFrame_.isValid())
      {
        const openni::RGB888Pixel* colorImagePix = (const openni::RGB888Pixel*)colorFrame_.getData();
        memcpy(colorImage_.data,colorImagePix,colorImage_.cols*colorImage_.rows*3*sizeof(unsigned char));
        return true;
      }
      else
      {
        if (verbose_)
          std::cout << "[OpenNICapture::retrieveImage] color frame wrong!" << std::endl;
        return false;
      }
    }
    
    case openni::SENSOR_DEPTH:
    {
      depthStream_.readFrame(&depthFrame_);
      if (depthFrame_.isValid())
      {
        const openni::DepthPixel* depthImagePix = (const openni::DepthPixel*)depthFrame_.getData();
        memcpy(depthImage_.data,depthImagePix,depthImage_.cols*depthImage_.rows*sizeof(openni::DepthPixel));
      
        // Update queue
        while (depthImages_.size() >= static_cast<size_t>(depthSmoothingNumFrames_))
          depthImages_.pop_front();
        depthImages_.push_back(depthImage_.clone());
        
        return true;
      }
      else
      {
        if (verbose_)
          std::cout << "[OpenNICapture::retrieveImage] depth frame wrong!" << std::endl;
        return false;
      }
    }

    case openni::SENSOR_IR:
    {
      IRStream_.readFrame(&IRFrame_);
      if (IRFrame_.isValid())
      {
        const openni::Grayscale16Pixel* irImagePix = (const openni::Grayscale16Pixel*)IRFrame_.getData();
        memcpy(IRImage_.data, irImagePix, IRImage_.cols*IRImage_.rows * sizeof(openni::Grayscale16Pixel));
        return true;
      }
      else
      {
        if (verbose_)
          std::cout << "[OpenNICapture::retrieveImage] IR frame wrong!" << std::endl;
        return false;
      }      
    }
    
    default:
    {
      std::cout << "[OpenNICapture::retrieveImage] unknown sensor type."  << std::endl;
      std::cout << "[OpenNICapture::retrieveImage] " << sensor_type << std::endl;
      return false;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
char OpenNICapture::retrieveVisualize(const CAPTURE_MODE capture_mode)
{  
  //--------------------------------------------------------------------------
  // Retrieve frames
  
  openni::Status rc = openni::STATUS_OK;
  int updatedStreamIndex = -1;
  bool depthUpdated = false;
  bool colorUpdated = false;
  bool IRUpadted = false;
      
  while (rc == openni::STATUS_OK)
  {
    rc = openni::OpenNI::waitForAnyStream(streams_, 3, &updatedStreamIndex, 0);
    if (rc == openni::STATUS_OK)
    {        
      switch (updatedStreamIndex)
      {
      case 0:
        retrieveImage(openni::SENSOR_DEPTH);
        depthUpdated = true;
        break;
      case 1:
        retrieveImage(openni::SENSOR_COLOR);
        colorUpdated = true;
        break;
      case 2:
        retrieveImage(openni::SENSOR_IR);
        IRUpadted = true;
        break;
      }
    }
  }
  
  //--------------------------------------------------------------------------
  // Update display

  // Depth
  if (depthUpdated && (capture_mode == DEPTH || capture_mode == DEPTH_RGB || capture_mode == DEPTH_IR || capture_mode == DEPTH_RGB_IR))
  {
    if (!depthSmoothing_)
      depthImageSave_ = depthImages_.back();
    else
      depthImageSave_ = utl::kinect::depthImageTemporalSmoothing(depthImages_);
    
    depthImageDisplay_ = depthImageSave_.clone();
    depthImageDisplay_ = depthImageDisplay_ / 6000 * std::numeric_limits<unsigned short>::max();
    depthImageDisplay_ = utl::ocv::resize(depthImageDisplay_, displayImageWidth_);        
    
    if (showCaptureInfo_)
      printCaptureInfo(depthImageDisplay_);
    if (capturingOniFile_)
      addRecordDisplay(depthImageDisplay_);      
    cv::imshow("Depth", depthImageDisplay_);
  }
  
  // Color
  if (colorUpdated && (capture_mode == RGB || capture_mode == DEPTH_RGB || capture_mode == DEPTH_RGB_IR))
  {
    // Update visualization
    cv::cvtColor(colorImage_, colorImage_, CV_RGB2BGR);
    colorImageDisplay_ = colorImage_.clone();
    colorImageDisplay_ = utl::ocv::resize(colorImageDisplay_, displayImageWidth_);        

    // Display
    if (showCaptureInfo_)
      printCaptureInfo(colorImageDisplay_);
    if (capturingOniFile_)
      addRecordDisplay(colorImageDisplay_);
    cv::imshow("RGB", colorImageDisplay_);
  }
  
  // IR
  if (IRUpadted && (capture_mode == IR || capture_mode == DEPTH_IR || capture_mode == RGB_IR || capture_mode == DEPTH_RGB_IR))
  {
    // Update visualization
    IRImageDisplay_ = IRImage_.clone();
    IRImageDisplay_ = IRImageDisplay_ * 300;
    IRImageDisplay_ = utl::ocv::resize(IRImageDisplay_, displayImageWidth_);
            
    // Display
    if (showCaptureInfo_)
      printCaptureInfo(IRImageDisplay_);
    if (capturingOniFile_)
      addRecordDisplay(IRImageDisplay_);      
    cv::imshow("IR", IRImageDisplay_);
  }
  
  return cv::waitKey(1);
}

////////////////////////////////////////////////////////////////////////////////
bool OpenNICapture::captureImages()
{
  bool success = true;
  if (captureMode_ == RGB || captureMode_ == DEPTH_RGB)
    success &= utl::kinect::writeFrame(captureDirname_, curFrameIndex_, utl::kinect::RGB, colorImage_, captureFilenamePrefix_);

  if (captureMode_ == DEPTH || captureMode_ == DEPTH_RGB)
    success &= utl::kinect::writeFrame(captureDirname_, curFrameIndex_, utl::kinect::DEPTH, depthImageSave_, captureFilenamePrefix_);

  if (captureMode_ == IR)
    success &= utl::kinect::writeFrame(captureDirname_, curFrameIndex_, utl::kinect::IR, IRImage_, captureFilenamePrefix_);
  
  if (captureMode_ == RGB_IR)
  {
    success &= utl::kinect::writeFrame(captureDirname_, curFrameIndex_, utl::kinect::IR, IRImage_, captureFilenamePrefix_);
    startStream(openni::SENSOR_COLOR);
    int delay = 1;
    auto t_start  = std::chrono::high_resolution_clock::now();
    auto t_end    = std::chrono::high_resolution_clock::now();
    while (std::chrono::duration<double>(t_end-t_start).count() < (double)delay)
    {
      retrieveVisualize(IR);
      t_end    = std::chrono::high_resolution_clock::now();
    }

    retrieveImage(openni::SENSOR_COLOR);
    cv::imshow("RGB", colorImageDisplay_);
    success &= utl::kinect::writeFrame(captureDirname_, curFrameIndex_, utl::kinect::RGB, colorImage_, captureFilenamePrefix_);
    startStream(openni::SENSOR_IR);
  }
  
  if (captureMode_ == DEPTH_IR)
  {
    bool irEmitterStatus = getIREmitterStatus();
    
    // Capture depth
    if (!getIREmitterStatus())
    {
      setIREmitterStatus(true);
      
      int delay = 1;
      auto t_start  = std::chrono::high_resolution_clock::now();
      auto t_end    = std::chrono::high_resolution_clock::now();      
      while (std::chrono::duration<double>(t_end-t_start).count() < (double)delay)
      {
        int updatedStreamIndex;
        openni::Status rc = openni::OpenNI::waitForAnyStream(streams_, 3, &updatedStreamIndex, 1000);
        retrieveImage(openni::SENSOR_IR);
        retrieveImage(openni::SENSOR_DEPTH);
        cv::imshow("IR", IRImageDisplay_);
        cv::imshow("Depth", depthImageDisplay_);
        cv::waitKey(30);
        t_end    = std::chrono::high_resolution_clock::now();
      }      
    }
    
    success &= utl::kinect::writeFrame(captureDirname_, curFrameIndex_, utl::kinect::DEPTH, depthImageSave_, captureFilenamePrefix_);

    // Capture IR
    if (getIREmitterStatus())
    {
      setIREmitterStatus(false);
      
      int delay = 1;
      auto t_start  = std::chrono::high_resolution_clock::now();
      auto t_end    = std::chrono::high_resolution_clock::now();
      while (std::chrono::duration<double>(t_end-t_start).count() < (double)delay)
      {
        retrieveVisualize(DEPTH_IR);
        t_end    = std::chrono::high_resolution_clock::now();
      }
    }
    
    success &= utl::kinect::writeFrame(captureDirname_, curFrameIndex_, utl::kinect::IR, IRImage_, captureFilenamePrefix_);
    
    // Return emitter to original status
    if (getIREmitterStatus() != irEmitterStatus)
      setIREmitterStatus(irEmitterStatus);
  }
  
  if (captureMode_ == DEPTH_RGB_IR)
  {
    bool irEmitterStatus = getIREmitterStatus();
    
    // Capture RGB
    success &= utl::kinect::writeFrame(captureDirname_, curFrameIndex_, utl::kinect::RGB, colorImage_, captureFilenamePrefix_);
    
    // Capture depth
    startStream(openni::SENSOR_IR);
    if (!getIREmitterStatus())
    {
      setIREmitterStatus(true);
      
      int delay = 1;
      auto t_start  = std::chrono::high_resolution_clock::now();
      auto t_end    = std::chrono::high_resolution_clock::now();      
      while (std::chrono::duration<double>(t_end-t_start).count() < (double)delay)
      {
        retrieveVisualize(DEPTH_IR);
        t_end    = std::chrono::high_resolution_clock::now();
      }      
    }
    
    success &= utl::kinect::writeFrame(captureDirname_, curFrameIndex_, utl::kinect::DEPTH, depthImageSave_, captureFilenamePrefix_);

    // Capture IR
    if (getIREmitterStatus())
    {
      setIREmitterStatus(false);
      
      int delay = 1;
      auto t_start  = std::chrono::high_resolution_clock::now();
      auto t_end    = std::chrono::high_resolution_clock::now();
      while (std::chrono::duration<double>(t_end-t_start).count() < (double)delay)
      {
        retrieveVisualize(DEPTH_IR);
        t_end    = std::chrono::high_resolution_clock::now();
      }
    }
    
    success &= utl::kinect::writeFrame(captureDirname_, curFrameIndex_, utl::kinect::IR, IRImage_, captureFilenamePrefix_);
        
    // Get back to original state
    updateStreams();
    if (getIREmitterStatus() != irEmitterStatus)
      setIREmitterStatus(irEmitterStatus);
    
  }
  
  if (success)
  {
    std::cout << "Frame " << curFrameIndex_ << " captured!" << std::endl;
    curFrameIndex_++;
  }
  return success;
}

////////////////////////////////////////////////////////////////////////////////
void OpenNICapture::startOniCapture(const int delay)
{
  // Check if we are already capturing
  if (capturingOniFile_)
  {
    std::cout << "[OpenNICapture::startOniCapture] Already capturing an oni file." << std::endl;
    return;
  }
  
  // Figure out which streams to capture
  std::vector<int> captureStreams;
  
  switch (captureMode_)
  {
    case RGB:
      captureStreams.push_back(1);
      break;
      
    case DEPTH:
      captureStreams.push_back(0);
      break;
      
    case IR:
      captureStreams.push_back(2);
      break;
      
    case RGB_IR:
      std::cout << "[OpenNICapture::startOniCapture] Cannot capture oni sequences in IR_RGB mode" << std::endl;
      return;
      
    case DEPTH_RGB:
      captureStreams.push_back(0);
      captureStreams.push_back(1);
      break;
      
    case DEPTH_IR:
      captureStreams.push_back(0);
      captureStreams.push_back(2);
      break;
      
    case DEPTH_RGB_IR:
      std::cout << "[OpenNICapture::startOniCapture] Cannot capture oni sequences in DEPTH_RGB_IR mode" << std::endl;
      return;
      
    default:
      std::cout << "[OpenNICapture::startOniCapture] Unknown capture mode" << std::endl;
      return;
  }
    
  // Create recorder
  openni::Status rc;
  std::string oniFilename = generateOniFilename();
  rc = recorder_.create(oniFilename.c_str());
  if (rc != openni::STATUS_OK)
  {
    std::cout << openni::OpenNI::getExtendedError() << std::endl;
    std::abort();
  }
  
  // Add streams to recorder
  for (auto streamIt = captureStreams.begin(); streamIt != captureStreams.end(); streamIt++)
  {
    bool lossyCompression;
    switch (*streamIt)
    {
      case 0:
        lossyCompression = depthLossyCompression_;  break;
      case 1:
        lossyCompression = colorLossyCompression_;  break;
      case 2:
        lossyCompression = IRLossyCompression_;  break;
    }
    
    rc = recorder_.attach(*streams_[*streamIt], lossyCompression);
    if (rc != openni::STATUS_OK)
    {
      std::cout << openni::OpenNI::getExtendedError() << std::endl;
      std::abort();
    }
  }
  
  // Do the delay
  auto t_start  = std::chrono::high_resolution_clock::now();
  auto t_end    = std::chrono::high_resolution_clock::now();
  while (std::chrono::duration<double>(t_end-t_start).count() < (double)delay)
  {
    retrieveVisualize(captureMode_);
    t_end    = std::chrono::high_resolution_clock::now();
  }
  
  // Start recording
  rc = recorder_.start();
  if (rc != openni::STATUS_OK)
  {
    std::cout << openni::OpenNI::getExtendedError() << std::endl;
    std::abort();
  }
  capturingOniFile_ = true;
}

////////////////////////////////////////////////////////////////////////////////
void OpenNICapture::stopOniCapture()
{
  if (capturingOniFile_)
  {
    recorder_.destroy();
    std::cout << "oni sequence " << curOniIndex_++ << " captured!" << std::endl;
    capturingOniFile_ = false;
  }
}

////////////////////////////////////////////////////////////////////////////////
void OpenNICapture::printCaptureInfo(cv::Mat &image_display)
{
  float textFontIntensity;
  switch (image_display.depth())
  {
    case CV_8U:
      textFontIntensity = 255.0f;
      break;
      
    case CV_16U:
      textFontIntensity = std::pow(2, 16);
      break;
      
    default:
      textFontIntensity = 255.0f;
  }
    
  // Add text
  std::string text0 = "Capture mode: ";
  switch (captureMode_)
  {
    case RGB:
      text0 += "RGB";
      break;
      
    case DEPTH:
      text0 += "Depth";
      break;
      
    case IR:
      text0 += "IR";
      break;
      
    case RGB_IR:
      text0 += "IR + RGB";
      break;
      
    case DEPTH_RGB:
      text0 += "Depth + RGB";
      break;
      
    case DEPTH_IR:
      text0 += "Depth + IR";
      break;
      
    case DEPTH_RGB_IR:
      text0 += "Depth + IR + RGB";
      break;
      
    default:
      text0 += "unknown";
  }
    
  std::string text1 = "Close range depth: ";
  if (getDepthCloseRangeMode())
    text1 += "ON";
  else
    text1 += "OFF";
  
  std::string text2 = "Depth registration: ";
  if (getDepthToColorRegistration())
    text2 += "ON";
  else
    text2 += "OFF";

  std::string text3 = "Depth smoothing: ";
  if (depthSmoothing_)
    text3 += "ON " + std::to_string(depthSmoothingNumFrames_) + " frames";
  else
    text3 += "OFF";
  
  std::string text4 = "IR emitter: ";
  if (irEmitterStatus_)
    text4 += "ON";
  else
    text4 += "OFF";

  std::string text5 = "Depth lossy compression: ";
  if (depthLossyCompression_)
    text5 += "ON";
  else
    text5 += "OFF";  
  
  std::string text6 = "RGB lossy compression: ";
  if (colorLossyCompression_)
    text6 += "ON";
  else
    text6 += "OFF";

  std::string text7 = "IR lossy compression: ";
  if (IRLossyCompression_)
    text7 += "ON";
  else
    text7 += "OFF";  

  std::string text8 = "Depth RGB frame sync: ";
  if (getDepthColorFrameSync())
    text8 += "ON";
  else
    text8 += "OFF";
  
  int fontFace = cv::FONT_HERSHEY_SIMPLEX;
  double fontScale = 0.7;
  int thickness = 2;
  cv::Scalar fontColor (textFontIntensity, textFontIntensity, textFontIntensity);
  cv::Point textOrg(20, image_display.rows - 250);
  cv::putText(image_display, text0, textOrg, fontFace, fontScale, fontColor, thickness, 8);
  textOrg.y += 30;
  cv::putText(image_display, text1, textOrg, fontFace, fontScale, fontColor, thickness, 8);
  textOrg.y += 30;
  cv::putText(image_display, text2, textOrg, fontFace, fontScale, fontColor, thickness, 8);
  textOrg.y += 30;
  cv::putText(image_display, text3, textOrg, fontFace, fontScale, fontColor, thickness, 8);
  textOrg.y += 30;  
  cv::putText(image_display, text4, textOrg, fontFace, fontScale, fontColor, thickness, 8);
  textOrg.y += 30;  
  cv::putText(image_display, text5, textOrg, fontFace, fontScale, fontColor, thickness, 8);  
  textOrg.y += 30;  
  cv::putText(image_display, text6, textOrg, fontFace, fontScale, fontColor, thickness, 8);  
  textOrg.y += 30;  
  cv::putText(image_display, text7, textOrg, fontFace, fontScale, fontColor, thickness, 8);  
  textOrg.y += 30;  
  cv::putText(image_display, text8, textOrg, fontFace, fontScale, fontColor, thickness, 8);  
}

////////////////////////////////////////////////////////////////////////////////
void OpenNICapture::addRecordDisplay(cv::Mat& display_image)
{
  float intensity;
  
  switch (display_image.type())
  {
    case CV_8UC3:
      intensity = 255.0f;
      break;
      
    case CV_16U:
      cv::cvtColor(display_image, display_image, CV_GRAY2BGR);
      intensity = std::pow(2, 16);
      break;
      
    default:
      std::cout << "[OpenNICapture::addRecordDisplay] unexpected image type." << std::endl;
      std::cout << display_image.type() << std::endl;
      std::abort();
  }
  
  // Draw circle
  int circleRadius = 20;
  cv::Point circleCenter (display_image.cols - 100, display_image.rows - 100);
  cv::circle(display_image, circleCenter, circleRadius, cv::Scalar(0.0, 0.0, intensity), CV_FILLED);
}

////////////////////////////////////////////////////////////////////////////////
std::string OpenNICapture::generateOniFilename()
{
  std::stringstream num;
  num << std::setfill('0') << std::setw(3) << curOniIndex_;
  
  if (captureFilenamePrefix_ != "")
    return utl::fs::fullfile(captureDirname_, captureFilenamePrefix_ + "_" + num.str() + ".oni");
  else
    return utl::fs::fullfile(captureDirname_, num.str() + ".oni");
}

////////////////////////////////////////////////////////////////////////////////
int OpenNICapture::getOniFilenameIndex  ( const std::string &oni_filename)
{
  int startPos = captureFilenamePrefix_.length() + 1;
  int length = 3;
  
  if (oni_filename.length() < oni_filename.length())
    return -1;
  
  std::string id_str = oni_filename.substr(startPos, length);
  
  // Check if substring is a number
  bool isNumber = true;
  for (size_t cId = 0; cId < length; cId++)
    if (!std::isdigit(id_str[cId]))
    {
      isNumber = false;
      break;
    }
    
  if (!isNumber)
    return -1;
  
  return std::stoi(id_str);
}