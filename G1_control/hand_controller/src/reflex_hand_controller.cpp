/*
 * =====================================================================================
 *
 *       Filename:  reflex_hand_controller.cpp
 *
 *    Description:  Action primitive control for reflex hand 
 *
 *        Version:  1.0
 *        Created:  06/20/2016 05:36:00 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Bing Ai, bing.ai@utexas.edu
 *   Organization:  
 *
 * =====================================================================================
 */

#include <g1_control/hand_controller.h>
//#include <std_srvs/Empty.h>
#include <reflex_msgs/SetTactileThreshold.h>
#include <reflex_msgs/SetTactileThresholdRequest.h>
#include <reflex_msgs/Command.h>
#include <reflex_msgs/PoseCommand.h>
#include <reflex_msgs/VelocityCommand.h>
#include <reflex_msgs/ForceCommand.h>
#include <reflex_msgs/FingerPressure.h>
#include <boost/array.hpp>
#include <std_srvs/Empty.h>
#include <cmath>

namespace g1

{

namespace control

{

ReflexHandController::ReflexHandController(ros::NodeHandle &handle)
: HandController("reflex")

{
  // Services can automatically call hand calibration
  calibrate_fingers_service_ = handle.serviceClient<std_srvs::Empty>("reflex_takktile/calibrate_fingers");
  calibrate_tactile_service_ = handle.serviceClient<std_srvs::Empty>("reflex_takktile/calibrate_tactile");

  // Services can set tactile thresholds and enable tactile stops
  enable_tactile_stops_service_ = handle.serviceClient<std_srvs::Empty>("reflex_takktile/enable_tactile_stops");
  disable_tactile_stops_service_ = handle.serviceClient<std_srvs::Empty>("reflex_takktile/disable_tactile_stops");
  set_tactile_threshold_service_ = handle.serviceClient<reflex_msgs::SetTactileThreshold>("reflex_takktile/set_tactile_threshold");

  //This collection of publishers can be used to command the hand
  command_pub_ = handle.advertise<reflex_msgs::Command>("reflex_takktile/command", 1);
  pos_pub_ = handle.advertise<reflex_msgs::PoseCommand>("reflex_takktile/command_position",1);
  vel_pub_ = handle.advertise<reflex_msgs::VelocityCommand>("reflex_takktile/command_velocity",1);
  force_pub_ = handle.advertise<reflex_msgs::ForceCommand>("reflex_takktile/command_motor_force",1); 

  //rospy.Subscriber('reflex_takktile/hand_state', Hand, hand_state_cb)
  reflex_state_sub_ = handle.subscribe("reflex_takktile/hand_state", 1, &ReflexHandController::hand_state_callback, this);
}

ReflexHandController::~ReflexHandController()
{}

bool ReflexHandController::calibrate_fingers_()
{
  std_srvs::Empty srv;
  bool result = calibrate_fingers_service_.call(srv);
  if (result) {
    ros::Duration(4).sleep();
    // sleep_();
    ROS_INFO("Finger calibration is done!");
  }
  return result;
}

bool ReflexHandController::calibrate_tactile_()
{
  std_srvs::Empty srv;
  bool result = calibrate_tactile_service_.call(srv);
  if (result){
    ros::Duration(1).sleep();
    // sleep_();
    ROS_INFO("Tactile calibration is done!");
  }
  return result;
}

bool ReflexHandController::enable_tactile_stops_()
{
  std_srvs::Empty srv;
  bool result = enable_tactile_stops_service_.call(srv);
  if (result){
    ros::Duration(1).sleep();
    // sleep_();  
    ROS_INFO("Tactile_stops enabled!");
  }
  return result;
}

bool ReflexHandController::disable_tactile_stops_()
{
  std_srvs::Empty srv;
  bool result = disable_tactile_stops_service_.call(srv);
  if (result){
    ros::Duration(1).sleep();
    // sleep_();
    ROS_INFO("Tactile_stops disabled!");
  }
  return result;
}

void ReflexHandController::hand_state_callback(const reflex_msgs::Hand& data)
{
  reflex_hand_state_ = data;
  // ROS_INFO("Callback done!");
}


G1ControlErrorCode ReflexHandController::reset_(double f1, double f2, double f3, double preshape)
{
  
  open_Fingers_(f1, f2, f3);
  sleep_();
  double threshold = 0.05;
  while(reflex_hand_state_.motor[0].joint_angle > threshold || 
        reflex_hand_state_.motor[1].joint_angle > threshold ||
        reflex_hand_state_.motor[2].joint_angle > threshold){
  open_Fingers_(f1, f2, f3);
  sleep_();
  ROS_INFO_STREAM("Motor info: " << reflex_hand_state_.motor[0].joint_angle << ", " << reflex_hand_state_.motor[1].joint_angle << ", " << reflex_hand_state_.motor[2].joint_angle);
  }

  open_Preshape_(preshape);
  sleep_();
  while(reflex_hand_state_.motor[3].joint_angle > threshold){
  open_Preshape_(preshape);
  sleep_();
  ROS_INFO_STREAM("Motor info: "<< reflex_hand_state_.motor[3].joint_angle);
  }
  
  return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::SUCCESS);

}


// G1ControlErrorCode ReflexHandController::close_(double f1, double f2, double f3)
// {
//   G1ControlErrorCode result(g1_control_msgs::G1ControlErrorCodes::FAILURE);
//   reflex_msgs::ForceCommand  forcecmd;
//   forcecmd.f1 = f1;
//   forcecmd.f2 = f2;
//   forcecmd.f3 = f3;
//   force_pub_.publish(forcecmd);
//   // ros::Duration(2).sleep();
//   sleep_();
//   return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::SUCCESS);
// }

G1ControlErrorCode ReflexHandController::close_(int grasp_type)
{
  G1ControlErrorCode result(g1_control_msgs::G1ControlErrorCodes::FAILURE);
  double f1 = 150;
  double f2 = 150;
  double f3;
  if (grasp_type == 2){
     f3 = 0;
  }
  else {
    f3 = 300;
  }
  reflex_msgs::ForceCommand forcecmd;
  forcecmd.f1 = f1;
  forcecmd.f2 = f2;
  forcecmd.f3 = f3;
  force_pub_.publish(forcecmd);
  if (grasp_type == 2) {
    ros::Duration(1).sleep();
  }
  sleep_();

  if(grasp_type == 2){
    reflex_msgs::PoseCommand  posecmd;
    posecmd.f1 = reflex_hand_state_.motor[0].joint_angle;
    posecmd.f2 = reflex_hand_state_.motor[1].joint_angle;
    // posecmd.f3 = reflex_hand_state_.motor[2].joint_angle;
    posecmd.f3 = 1.5;
    posecmd.preshape = reflex_hand_state_.motor[3].joint_angle;
    pos_pub_.publish(posecmd);
    ros::Duration(1).sleep();
    sleep_(); 
  }
  return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::SUCCESS);
}


// G1ControlErrorCode ReflexHandController::close_pressure_(double f1, double f2, double f3, double preshape)
// {
//   G1ControlErrorCode result(g1_control_msgs::G1ControlErrorCodes::FAILURE);
  
//   enable_tactile_stops_();
//   reflex_msgs::FingerPressure fingercmd1;
//   boost::array<int, 9> fingerPressure1 = {{500, 500, 500, 500, 500, 15, 15, 15, 15}};
//   fingercmd1.sensor = fingerPressure1;

//   reflex_msgs::FingerPressure fingercmd2;
//   boost::array<int, 9> fingerPressure2 = {{500, 500, 500, 500, 500, 15, 15, 15, 15}};
//   fingercmd2.sensor = fingerPressure2;

//   reflex_msgs::FingerPressure fingercmd3;
//   boost::array<int, 9> fingerPressure3 = {{500, 500, 500, 500, 500, 30, 30, 30, 30}};
//   fingercmd3.sensor = fingerPressure3;

   
//   reflex_msgs::SetTactileThreshold threshold;
//   threshold.request.finger[0] = fingercmd1;
//   threshold.request.finger[1] = fingercmd2;
//   threshold.request.finger[2] = fingercmd3;

//   if (set_tactile_threshold_service_.call(threshold)) {
//     ROS_INFO("Set Threshold SUCCEED.");
//   }
//   else {
//     ROS_ERROR("Failed to call service SetTactileThreshold!");
//     return 1;
//   }


//   reflex_msgs::VelocityCommand velocitycmd;
//   velocitycmd.f1 = f1;
//   velocitycmd.f2 = f2;
//   velocitycmd.f3 = f3;
//   velocitycmd.preshape = preshape;
//   vel_pub_.publish(velocitycmd);
//   disable_tactile_stops_();
//   // ros::Duration(1).sleep();
//   sleep_();
//   return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::SUCCESS);
// }

G1ControlErrorCode ReflexHandController::close_pressure_(int grasp_type)
{
  G1ControlErrorCode result(g1_control_msgs::G1ControlErrorCodes::FAILURE);
  
  enable_tactile_stops_();
  reflex_msgs::FingerPressure fingercmd1;
  boost::array<int, 9> fingerPressure1 = {{500, 500, 500, 500, 500, 15, 15, 15, 15}};
  fingercmd1.sensor = fingerPressure1;

  reflex_msgs::FingerPressure fingercmd2;
  boost::array<int, 9> fingerPressure2 = {{500, 500, 500, 500, 500, 15, 15, 15, 15}};
  fingercmd2.sensor = fingerPressure2;

  
  reflex_msgs::FingerPressure fingercmd3;
  boost::array<int, 9> fingerPressure3;
  if(grasp_type == 2){
    boost::array<int, 9> fingerPressure3 = {{0, 0, 0, 0, 0, 0, 0, 0, 0}};
  }
  else{
    boost::array<int, 9> fingerPressure3 = {{500, 500, 500, 500, 500, 30, 30, 30, 30}};
  }
  fingercmd3.sensor = fingerPressure3;

   
  reflex_msgs::SetTactileThreshold threshold;
  threshold.request.finger[0] = fingercmd1;
  threshold.request.finger[1] = fingercmd2;
  threshold.request.finger[2] = fingercmd3;

  if (set_tactile_threshold_service_.call(threshold)) {
    ROS_INFO("Set Threshold SUCCEED.");
  }
  else {
    ROS_ERROR("Failed to call service SetTactileThreshold!");
    return 1;
  }


  // reflex_msgs::VelocityCommand velocitycmd;
  // velocitycmd.f1 = f1;
  // velocitycmd.f2 = f2;
  // velocitycmd.f3 = f3;
  // velocitycmd.preshape = preshape;
  // vel_pub_.publish(velocitycmd);
  disable_tactile_stops_();
  // ros::Duration(1).sleep();
  sleep_();
  return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::SUCCESS);
}


G1ControlErrorCode ReflexHandController::open_Fingers_(double f1, double f2, double f3)
{
  G1ControlErrorCode result(g1_control_msgs::G1ControlErrorCodes::FAILURE);
  reflex_msgs::PoseCommand  posecmd;
  posecmd.f1 = f1;
  posecmd.f2 = f2;
  posecmd.f3 = f3;
  posecmd.preshape = reflex_hand_state_.motor[3].joint_angle;
  pos_pub_.publish(posecmd);
  // ros::Duration(2).sleep();
  sleep_(); 
  return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::SUCCESS);
}


G1ControlErrorCode ReflexHandController::open_Preshape_(double preshape)
{
  G1ControlErrorCode result(g1_control_msgs::G1ControlErrorCodes::FAILURE);
  reflex_msgs::PoseCommand  posecmd;
  posecmd.f1 = reflex_hand_state_.motor[0].joint_angle;
  posecmd.f2 = reflex_hand_state_.motor[1].joint_angle;
  posecmd.f3 = reflex_hand_state_.motor[2].joint_angle;
  posecmd.preshape = preshape;
  pos_pub_.publish(posecmd);
  // ros::Duration(2).sleep();
  sleep_(); 
  return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::SUCCESS);
}


G1ControlErrorCode ReflexHandController::open_(double f1, double f2, double f3, double preshape)
{
  G1ControlErrorCode result(g1_control_msgs::G1ControlErrorCodes::FAILURE);
  open_Fingers_(f1, f2, f3);
  open_Preshape_(preshape);
  return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::SUCCESS);
}


// get the grasp type for the object
// grasp_type = 0: wrap grasp with Preshape = 0; Horizontal; example: mug #1 
// grasp_type = 1: pinch grasp (three fingers) with Preshape = pi; Horizontal; example: bowl #1 the third finger will close
// grasp_type = 2: pinch grasp (three fingers) with Preshape = pi; Horizontal; example: small(short) mug #? the third finger will NOT close
// grasp_type = 3: power grasp (three fingers) with Preshape = 2*pi/3; vertical; example: ball

G1ControlErrorCode ReflexHandController::preGrasp_(int type)
{
  G1ControlErrorCode result(g1_control_msgs::G1ControlErrorCodes::FAILURE);

  reflex_msgs::PoseCommand  posecmd;
  
  // grasp_type = 0: wrap grasp with Preshape = 0; Horizontal; example: mug #1 
  if (type == 0 || type == 4) {
    ROS_INFO("Grasp type 0: Wrap grasp with Preshape = 0; Horizontal");
    open_Preshape_(0);
    open_Fingers_(1,1,1.3);
  }

  // grasp_type = 1: pinch grasp (three fingers) with Preshape = pi; Horizontal; example: bowl #1 the third finger will close
  else if (type == 1) {
    ROS_INFO("Grasp type 1: Pinch grasp (three fingers) with Preshape = pi; the third finger will close; Horizontal");
    open_Preshape_(1.57);
    open_Fingers_(1,1,1.3);
  }

  // grasp_type = 2: pinch grasp (three fingers) with Preshape = pi; Horizontal; example: small(short) mug #? the third finger will NOT close
  else if (type == 2) {
    ROS_INFO("Grasp type 2: Pinch grasp (three fingers) with Preshape = pi; the third finger will NOT close; Horizontal");
    open_Preshape_(1.57);
    open_Fingers_(1,1,1.3);
  }

  // grasp_type = 3: power grasp (three fingers) with Preshape = 2*pi/3; vertical; example: ball
  else {
    ROS_INFO("Grasp type 3: Power grasp (three fingers) with Preshape = 2*pi/3; vertical");
    open_Preshape_(1.05);
    open_Fingers_(1,1,1);
  }

  // ros::Duration(1).sleep();
  return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::SUCCESS);
}


G1ControlErrorCode ReflexHandController::preOpen_(int type)
{
  G1ControlErrorCode result(g1_control_msgs::G1ControlErrorCodes::FAILURE);
  float f1_offset=-0.5, f2_offset=-0.5, preshape_offset=0;
  float f3_offset;
  if (type == 0 || type == 1){
    f3_offset = -1.5;
  }
  else if(type == 3) {
    f3_offset = -0.5;
  }
  else {f3_offset = 0;}

  float preopen[] = {0, 0, 0, 0};
  float offset[] = {f1_offset, f2_offset, f3_offset, preshape_offset};
  for (int i = 0; i < 4; ++i){
    preopen[i] = reflex_hand_state_.motor[i].joint_angle + offset[i];
  } 
  // std::cout << preopen[0] << " "<< preopen[1] <<" "<<preopen[2]<<" "<<preopen[3]<<std::endl;
  
  reflex_msgs::PoseCommand  posecmd;
  posecmd.f1 = preopen[0];
  posecmd.f2 = preopen[1];
  posecmd.f3 = preopen[2];
  posecmd.preshape = preopen[3];
  pos_pub_.publish(posecmd);
  sleep_();
  return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::SUCCESS);
}


G1ControlErrorCode ReflexHandController::sleep_()
{
  float threshold = 0.08;
  float current_state[] = {reflex_hand_state_.motor[0].joint_angle, reflex_hand_state_.motor[1].joint_angle, reflex_hand_state_.motor[2].joint_angle, reflex_hand_state_.motor[3].joint_angle};
  ros::Duration(1).sleep();
  float motor_change_0 = std::abs(current_state[0] - reflex_hand_state_.motor[0].joint_angle);
  float motor_change_1 = std::abs(current_state[1] - reflex_hand_state_.motor[1].joint_angle);
  float motor_change_2 = std::abs(current_state[2] - reflex_hand_state_.motor[2].joint_angle);
  float motor_change_3 = std::abs(current_state[3] - reflex_hand_state_.motor[3].joint_angle);
  // ROS_INFO_STREAM("Motor first changes: " << motor_change_0 << ", " << motor_change_1 << ", " << motor_change_2 << ", " << motor_change_3);
  
  while(motor_change_0 > threshold || motor_change_1 > threshold || motor_change_2 > threshold || motor_change_3 > threshold){
    for(int i =0; i < 4; ++i){
      current_state[i] = reflex_hand_state_.motor[i].joint_angle;
    }
    ros::Duration(1).sleep();
    motor_change_0 = std::abs(current_state[0] - reflex_hand_state_.motor[0].joint_angle);
    motor_change_1 = std::abs(current_state[1] - reflex_hand_state_.motor[1].joint_angle);
    motor_change_2 = std::abs(current_state[2] - reflex_hand_state_.motor[2].joint_angle);
    motor_change_3 = std::abs(current_state[3] - reflex_hand_state_.motor[3].joint_angle);
    // ROS_INFO_STREAM("Motor changes: " << motor_change_0 << ", " << motor_change_1 << ", " << motor_change_2 << ", " << motor_change_3);
  }

  return true;
}

}
}


// int main(int argc, char** argv){
//   ros::init(argc, argv, "ReflexHandControllerTest");
//   ros::NodeHandle node;

//   ros::AsyncSpinner spinner(1);
//   spinner.start();

//   g1::control::HandController *reflexController = new g1::control::ReflexHandController(node);

//   ROS_INFO_STREAM("HandController type:" << reflexController->getType());

//   std::cout << "Calibration [y/N]? ";
//   char input;
//   std::cin >> input;
//   if (input == 'y'){
//   	reflexController->calibrate();
//     ROS_INFO("Calibrations are done!");
//   }

//   ROS_INFO("open...");
//   reflexController->open();

//   ROS_INFO("preGrasp...");
//   reflexController->preGrasp(2);
  
//   std::cout << "Close [y/N]? ";
//   std::cin >> input;
//   if (input == 'y'){
//     reflexController->close();
//   }

//   // ROS_INFO("close_pressure...");
//   // reflexController.close_pressure();
  

//   std::cout << "PreOpen [y/N]? ";
//   std::cin >> input;
//   if (input == 'y'){
//     reflexController->preOpen();
//     // ros::Duration(1).sleep();
//   }
 
//  std::cout << "Open [y/N]? ";
//   std::cin >> input;
//   if (input == 'y'){
//     reflexController->open();
//     // ros::Duration(1).sleep();
//   }
//   std::cout << "Reset [y/N]? ";
//   std::cin >> input;
//   if (input == 'y'){
//     reflexController->reset();
//   }

//   return 0;
// };
