#ifndef G1_CONTROL_HAND_CONTROLLER_
#define G1_CONTROL_HAND_CONTROLLER_


#include <g1_control_msgs/G1ControlErrorCodes.h>
#include <g1_control_msgs/G1ControlStateCodes.h>
#include <g1_control_msgs/error_code.h>
#include <ros/ros.h>
#include <reflex_msgs/Hand.h>
#include <baxter_core_msgs/EndEffectorState.h>

namespace g1
{

namespace control
{

class HandController
{
public:
  HandController(const std::string &type = "base", const std::string &side = "unknown") :  
  type_(type),
  side_(side),
  state_(0)
  {}

  ~HandController() {}

  std::string getType()
  {
    return type_;
  }

  std::string getSide(){
  	return side_;
  }

  int getState(){
    return state_;
  }

  virtual bool calibrate() = 0;
  virtual G1ControlErrorCode open() = 0;
  virtual G1ControlErrorCode close(int graspType) = 0;
  virtual G1ControlErrorCode preGrasp(int graspType) = 0;
  virtual G1ControlErrorCode preOpen(int graspType) = 0;
  virtual G1ControlErrorCode reset() = 0;


protected:
  std::string type_;
  std::string side_;
  int state_;

};


class ReflexHandController : public HandController
{
public:
  ReflexHandController(ros::NodeHandle &handle);
  ~ReflexHandController();

  bool calibrate() 
  {
    return calibrate_fingers_() && calibrate_tactile_();
  }

  G1ControlErrorCode open()
  { 
    state_ = 1;
    return open_(); 
  }

  G1ControlErrorCode close(int graspType)
  {
    state_ = 2;
    return close_(graspType);
  }

  G1ControlErrorCode preGrasp(int graspType)
  {
    state_ = 3;
    return preGrasp_(graspType);
  }

  G1ControlErrorCode preOpen(int graspType)
  {
    state_ = 4;
    return preOpen_(graspType);
  }
    G1ControlErrorCode reset()
  {
    state_ = 5;
    return reset_();
  }

   void hand_state_callback(const reflex_msgs::Hand &data);
  //This collection of publishers can be used to command the hand
  
  // G1ControlErrorCode preOpen(boost::array<double, 4> offset =boost::array<double, 4>({{-1,-1,-1.5,0}}));

private:
  G1ControlErrorCode reset_(double f1=0, double f2=0, double f3=0, double preshape=0);
  // G1ControlErrorCode close_(double f1=150, double f2=150, double f3=300);
  G1ControlErrorCode close_(int graspType);
  G1ControlErrorCode open_(double f1=2.0, double f2=2.0, double f3=2.0, double preshape=2);
  G1ControlErrorCode open_Preshape_(double preshape);
  G1ControlErrorCode open_Fingers_(double f1, double f2, double f3);
  G1ControlErrorCode preGrasp_(int graspType);
  G1ControlErrorCode preOpen_(int graspType);
  // G1ControlErrorCode close_pressure_(double f1=0.50, double f2=0.50, double f3=0.50, double preshape=0.0);
  G1ControlErrorCode close_pressure_(int graspType);
  G1ControlErrorCode sleep_();


  bool calibrate_fingers_();  
  bool calibrate_tactile_();
  bool enable_tactile_stops_();
  bool disable_tactile_stops_();
  // Services can automatically call hand calibration
  ros::ServiceClient calibrate_fingers_service_;
  ros::ServiceClient calibrate_tactile_service_;

  // Services can set tactile thresholds and enable tactile stops
  ros::ServiceClient enable_tactile_stops_service_;
  ros::ServiceClient disable_tactile_stops_service_;
  ros::ServiceClient set_tactile_threshold_service_;

  //This collection of publishers can be used to command the hand
  ros::Publisher command_pub_;
  ros::Publisher pos_pub_;
  ros::Publisher vel_pub_;
  ros::Publisher force_pub_;

  reflex_msgs::Hand reflex_hand_state_;
  ros::Subscriber reflex_state_sub_;

};

class BaxterGripperController : public HandController
{
public:
  BaxterGripperController(ros::NodeHandle &handle, const std::string &side);
  ~BaxterGripperController();

  bool calibrate() 
  {
    return calibrate_();
  }

  G1ControlErrorCode open()
  {
    state_ = 1;
    return open_(); 
  }

  G1ControlErrorCode close(int graspType = 0)
  {
    return close_();
  }

  // grasp_type = 0: wrap grasp with Preshape = 0; Horizontal; example: mug #1 
  // grasp_type = 1: pinch grasp (three fingers) with Preshape = pi; Horizontal; example: bowl #1 the third finger will close
  // grasp_type = 2: pinch grasp (three fingers) with Preshape = pi; Horizontal; example: small(short) mug #? the third finger will NOT close
  // grasp_type = 3: power grasp (three fingers) with Preshape = 2*pi/3; vertical; example: ball
  G1ControlErrorCode preGrasp(int graspType = 0)
  {
    return g1_control_msgs::G1ControlErrorCodes::SUCCESS;
  }

  G1ControlErrorCode preOpen(int graspType = 0)
  {
    return g1_control_msgs::G1ControlErrorCodes::SUCCESS;
  }
    G1ControlErrorCode reset()
  {
    return open_();
  }

  void gripper_state_callback(const baxter_core_msgs::EndEffectorState& data);


private:
  G1ControlErrorCode close_();
  G1ControlErrorCode open_();
  bool calibrate_();  

  //This collection of publishers can be used to command the hand
  ros::Publisher gripper_cmd_;
  ros::Subscriber gripper_state_sub_;
  baxter_core_msgs::EndEffectorState gripper_state_;


};


}
}

#endif
