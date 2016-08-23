#ifndef DEPTH_IR_CALIBRATION_H
#define DEPTH_IR_CALIBRATION_H

#include <namaris/utilities/pcl_visualization.hpp>

// State
struct VisState
{
  VisState ()
    : cloudDisplayState_(NOTHING)
    , iterator_ (0)
    , dx_ (0)
    , dy_ (0)
    , pointSize_ (1.0)
    , updateDisplay_ (true)
    , coorectDepth_ (false)
    , saveResults_ (false)
  {};
  
  enum CloudDisplayState {NOTHING, CLOUD, CLOUD_IR_COLORED};
  
  CloudDisplayState cloudDisplayState_;
  int iterator_;
  int dx_;
  int dy_;
  float pointSize_;
  bool updateDisplay_;
  bool coorectDepth_;
  bool saveResults_;
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
    
    visState->dx_ = 0;
    visState->dy_ = 0;

    // Undistort depth
    if (key == "KP_0")
      visState->coorectDepth_ = !visState->coorectDepth_;    

    // Change displayed cloud
    else if (key == "KP_1")
      visState->cloudDisplayState_ = VisState::CLOUD;
    else if (key == "KP_2")
      visState->cloudDisplayState_ = VisState::CLOUD_IR_COLORED;
    else if (key == "KP_3")
      visState->cloudDisplayState_ = VisState::NOTHING;
    
    
    // Decentering
    else if (key == "Left")
      visState->dx_--;
    else if (key == "Right")
      visState->dx_++;
    else if (key == "Up")
      visState->dy_--;
    else if (key == "Down")
      visState->dy_++;

    // Frame iterator
    else if (key == "period")
      visState->iterator_++;
    else if (key == "comma")
      visState->iterator_--;
    
    // Point size
    else if (key == "KP_Add")
      visState->pointSize_ += 1.0;
    else if (key == "KP_Subtract")
      visState->pointSize_ = std::max(0.0, visState->pointSize_ - 1.0);    
    
    // Save results
    else if (key == "space")
      visState->saveResults_ = !visState->saveResults_;
    
    else
      visState->updateDisplay_ = false;
  }
}

#endif    // DEPTH_IR_CALIBRATION_H