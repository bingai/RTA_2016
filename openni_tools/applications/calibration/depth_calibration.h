#ifndef DEPTH_CALIBRATION_H
#define DEPTH_CALIBRATION_H

// PCL
#include <namaris/utilities/pcl_visualization.hpp>

// State
struct VisState
{
  VisState ()
    : displayState_(ORIGINAL_DEPTH)
    , iterator_ (0)
    , showTarget_ (false)
    , updateDisplay_ (true)
  {};
  
  enum DisplayState   { ORIGINAL_DEPTH, UNDISTORTED_DEPTH };
  
  DisplayState displayState_;
  int iterator_;
  bool showTarget_;
  bool updateDisplay_;
};

// Callback
void keyboard_callback (const pcl::visualization::KeyboardEvent &event, void *cookie)
{
  VisState* visState = reinterpret_cast<VisState*> (cookie);
  
  if (event.keyUp ())
  {    
    std::string key = event.getKeySym ();
//     cout << key << " key pressed!\n";
    
    visState->updateDisplay_ = true;
    
    if ((key == "KP_1") || (key == "KP_End"))
      visState->displayState_ = VisState::ORIGINAL_DEPTH;
    else if ((key == "KP_2") || (key == "KP_Down"))
      visState->displayState_ = VisState::UNDISTORTED_DEPTH;
    
    else if (key == "Left")
      visState->iterator_--;
    else if (key == "Right")
      visState->iterator_++;

    else if ((key == "KP_0") || (key == "KP_Insert"))    
      visState->showTarget_ = !visState->showTarget_;
    else
      visState->updateDisplay_ = false;
  }
}

#endif    // DEPTH_CALIBRATION_H