#ifndef CALIBRATION_CHECK_H
#define CALIBRATION_CHECK_H

#include <namaris/utilities/pcl_visualization.hpp>

// State
struct VisState
{
  VisState ()
    : cloudDisplayState_(CLOUD)
    , iterator_ (0)
    , pointSize_ (1.0)
    , updateDisplay_ (true)
    , coorectDepth_ (false)
  {};
  
  enum CloudDisplayState {CLOUD, CLOUD_IR_COLORED, CLOUD_RGB_COLORED};
  
  CloudDisplayState cloudDisplayState_;
  int iterator_;
  float pointSize_;
  bool updateDisplay_;
  bool coorectDepth_;
};

// Callback
void keyboard_callback (const pcl::visualization::KeyboardEvent &event, void *cookie)
{
  VisState* visState = reinterpret_cast<VisState*> (cookie);
  
  if (event.keyUp ())
  {    
    std::string key = event.getKeySym ();
//     std::cout << key << " key pressed!\n";
    
    visState->updateDisplay_ = true;
    
    // Undistort depth
    if (key == "KP_0")
      visState->coorectDepth_ = !visState->coorectDepth_;    

    // Change displayed cloud
    else if (key == "KP_1")
      visState->cloudDisplayState_ = VisState::CLOUD;
    else if (key == "KP_2")
      visState->cloudDisplayState_ = VisState::CLOUD_IR_COLORED;
    else if (key == "KP_3")
      visState->cloudDisplayState_ = VisState::CLOUD_RGB_COLORED;
    
    // Frame iterator
    else if (key == "Right")
      visState->iterator_++;
    else if (key == "Left")
      visState->iterator_--;
    
    // Point size
    else if (key == "KP_Add")
      visState->pointSize_ += 1.0;
    else if (key == "KP_Subtract")
      visState->pointSize_ = std::max(0.0, visState->pointSize_ - 1.0);    
        
    else
      visState->updateDisplay_ = false;
  }
}

#endif    // CALIBRATION_CHECK_H