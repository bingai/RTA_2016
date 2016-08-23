#ifndef G1_CONTROL_ERROR_CODE_
#define G1_CONTROL_ERROR_CODE_

namespace g1
{
namespace control
{

class G1ControlErrorCode : public g1_control_msgs::G1ControlErrorCodes
{
public:
  G1ControlErrorCode() { val = 0; };
  G1ControlErrorCode(int code) { val = code; };
  G1ControlErrorCode(const g1_control_msgs::G1ControlErrorCodes &code) { val = code.val; };
  operator bool() const { return val == g1_control_msgs::G1ControlErrorCodes::SUCCESS; }
};

class G1ControlState : public g1_control_msgs::G1ControlStateCodes
{
public:
  G1ControlState() { val = 0; };
  G1ControlState(int state) { val = state; };
  G1ControlState(const g1_control_msgs::G1ControlStateCodes &state) { val = state.val; };
  bool operator ==(const g1_control_msgs::G1ControlStateCodes &state) const 
  {
    return val == state.val;
  } 
  
};

}
}

#endif

