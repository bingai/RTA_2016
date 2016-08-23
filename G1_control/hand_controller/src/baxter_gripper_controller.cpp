/*
  Desc:   Provides an action server for baxter's grippers
*/

// ROS
#include <g1_control/hand_controller.h>
// #include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <baxter_core_msgs/EndEffectorCommand.h>


namespace g1

{

namespace control

{


// static const std::string GRIPPER_COMMAND_ACTION_TOPIC="baxter_gripper_controller";

BaxterGripperController::BaxterGripperController(ros::NodeHandle &handle, const std::string &side)
: HandController("gripper", side)
{
  // Publisher
  std::string topic_name = "/robot/end_effector/" + side;
  gripper_cmd_ = handle.advertise<baxter_core_msgs::EndEffectorCommand>(topic_name + "_gripper/command",1);
  gripper_state_sub_ = handle.subscribe(topic_name + "_gripper/state", 1, &BaxterGripperController::gripper_state_callback, this);

}

BaxterGripperController::~BaxterGripperController()
{}

void BaxterGripperController::gripper_state_callback(const baxter_core_msgs::EndEffectorState& data)
{
  gripper_state_ = data;
  // ROS_INFO("Callback done!");
}

bool BaxterGripperController::calibrate_()
{
  baxter_core_msgs::EndEffectorCommand gripperMsg;
  gripperMsg.id = 65538;

  while (gripper_state_.calibrated != 0){
    gripperMsg.command = gripperMsg.CMD_CLEAR_CALIBRATION;
    gripperMsg.sender = "/gripper_grasp_node";
    gripperMsg.sequence = 3;
    gripper_cmd_.publish(gripperMsg);
  }

  gripperMsg.command = gripperMsg.CMD_CALIBRATE;
  gripperMsg.sender = "/gripper_grasp_node";
  gripperMsg.sequence = 3;
  gripper_cmd_.publish(gripperMsg);
  
  ros::Duration(4.0).sleep();
  ROS_INFO("Gripper calibration is done!");
  return true; 
}

G1ControlErrorCode BaxterGripperController::open_()
{
  G1ControlErrorCode result(g1_control_msgs::G1ControlErrorCodes::FAILURE);
  baxter_core_msgs::EndEffectorCommand gripperMsg;
  gripperMsg.id = 65538;
  gripperMsg.command = gripperMsg.CMD_GO;
  gripperMsg.args = "{\"position\": 100.0 }"; 
  gripperMsg.sender = "/gripper_grasp_node";
  gripperMsg.sequence = 3;
  ROS_INFO("Gripper open!");
  gripper_cmd_.publish(gripperMsg);

  ros::Duration(1).sleep();
  return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::SUCCESS);
}

G1ControlErrorCode BaxterGripperController::close_()
{
  G1ControlErrorCode result(g1_control_msgs::G1ControlErrorCodes::FAILURE);
  baxter_core_msgs::EndEffectorCommand gripperMsg;
  gripperMsg.id = 65538;
  gripperMsg.command = gripperMsg.CMD_GO;
  gripperMsg.args = "{ \"position\": 0.0 }"; 
  gripperMsg.sender = "/gripper_grasp_node";
  gripperMsg.sequence = 3;
  ROS_INFO("Gripper close!");
  gripper_cmd_.publish(gripperMsg);

  ros::Duration(1).sleep();
  return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::SUCCESS);
}

} // namespace control

} // namespace g1

