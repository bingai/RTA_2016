#ifndef FRAME_CAPTURE_H_
#define FRAME_CAPTURE_H_

// STD
#include <deque>

// OpenCV
#include <opencv2/core/core.hpp>

// OpenNI includes
#include <openni2/OpenNI.h>

// Project includes
#include "openni_tools/openni_tools.hpp"

class OpenNICapture
{
public:

  enum CAPTURE_MODE
  {
    DEPTH = '1',
    RGB = '2',
    IR = '3',
    DEPTH_RGB = '4',
    DEPTH_IR = '5',
    RGB_IR = '6',
    DEPTH_RGB_IR = '7'
  };

  // Constructor
  OpenNICapture(const CAPTURE_MODE &capture_mode,
                const std::string &capture_dirname = "",
                const std::string &capture_filename_prefix = "",
                const bool view_only = true,
                const bool verbose = false
  );

  // Destructor
  ~OpenNICapture();

  // Run
  void run();

private:

  // Capture parameters
  CAPTURE_MODE captureMode_;
  std::string captureDirname_;
  std::string captureFilenamePrefix_;
  int curFrameIndex_;
  int curOniIndex_;
  bool viewOnly_;
  int displayImageWidth_;

  // OpenNI stuff
  openni::Device device_;
  openni::VideoStream depthStream_;
  openni::VideoStream colorStream_;
  openni::VideoStream IRStream_;
  openni::VideoStream **streams_;

  openni::Recorder recorder_;

  openni::VideoFrameRef depthFrame_;
  openni::VideoFrameRef colorFrame_;
  openni::VideoFrameRef IRFrame_;

  openni::VideoMode depthMode_;
  openni::VideoMode colorMode_;
  openni::VideoMode IRMode_;

  openni::VideoMode colorModeLowRes_;
  openni::VideoMode colorModeHighRes_;
  openni::VideoMode IRModeLowRes_;
  openni::VideoMode IRModeHighRes_;

  bool colorStreamRunning_;
  bool depthStreamRunning_;
  bool IRStreamRunning_;

  bool depthLossyCompression_;
  bool colorLossyCompression_;
  bool IRLossyCompression_;
  bool depthColorFrameSync_;

  bool highResStreams_;
  bool depthCloseRangeMode_;
  bool irEmitterStatus_;
  bool depthToColorRegistration_;
  bool depthSmoothing_;
  int depthSmoothingNumFrames_;
  bool showCaptureInfo_;
  bool capturingOniFile_;

  cv::Mat colorImage_, depthImage_, IRImage_;
  cv::Mat colorImageDisplay_, depthImageDisplay_, IRImageDisplay_;
  cv::Mat depthImageSave_;
  std::deque<cv::Mat> depthImages_;

  bool verbose_;

  // Define available stream modes
  void defineStreamModes();

  // Update stream resolution
  void setStreamModes();

  // Initialize openni generators
  bool initStreams();

  // Start streams for a given sensor type
  void startStream(const openni::SensorType sensor_type);

  // Start streams for a given sensor type
  void stopStream(const openni::SensorType sensor_type);

  // Start streams for a given capture mode
  void updateStreams();

  // Check if depth to color registration is ON
  bool getDepthToColorRegistration();

  // Update depth registration
  void setDepthToColorRegistration(const bool new_value);

  // Check if depth close range is ON
  bool getDepthCloseRangeMode();

  // Update close range depth mode
  void setDepthCloseRangeMode(const bool new_value);

  // Get IR emitter status
  bool getIREmitterStatus();

  // Update IR emitter status
  void setIREmitterStatus(const bool new_value);

  // Get IR emitter status
  bool getDepthColorFrameSync();

  // Update IR emitter status
  void setDepthColorFrameSync(const bool new_value);

  // List availabel video modes for a given sensor info object
  void listAvailableModes(const openni::SensorInfo *sensorInfo);

  // List availabel video modes
  void listAvailableModes();

  // Retrieve an image from stream
  bool retrieveImage(const openni::SensorType sensor_type);

  // Run one iteration of retireval and visualization loop in the desired mode
  char retrieveVisualize(const CAPTURE_MODE capture_mode);

  // Capture images for current capture mode
  bool captureImages();

  // Start oni capture
  void startOniCapture(const int delay);

  // Stop oni capture
  void stopOniCapture();

  // Toggle depth color registration
  void toggleDepthColorRegistration();

  // Toggle depth close range mode
  void toggleCloseRangeDeph();

  // Toggle depth color synchronization
  void toggleFrameSync();

  // Print capture info on an image
  void printCaptureInfo(cv::Mat &display_image);

  // Add a record sign to the displayed image
  void addRecordDisplay(cv::Mat &display_image);

  // Generate oni filename
  std::string generateOniFilename();

  // Get index of the oni filename
  int getOniFilenameIndex(const std::string &oni_filename);

};

#endif  // FRAME_CAPTURE_H_