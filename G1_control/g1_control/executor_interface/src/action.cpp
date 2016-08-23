/*
 * =====================================================================================
 *
 *       Filename:  action.cpp
 *
 *    Description:  Action primitive for baxter 
 *
 *        Version:  1.0
 *        Created:  06/15/2016 02:24:00 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ren Mao (neroam), neroam@umd.edu
 *   Organization:  
 *
 * =====================================================================================
 */

#include <g1_control/action.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit/robot_state/conversions.h>

namespace g1
{

namespace control
{

// Move Action
Move::Move(const std::string &name)
  : Action(name, "Move"),
    joints_mode_(false),
    offset_mode_(false),
    left_start_idx_(0),
    right_start_idx_(0)
{
  retries_ = 2;
}

Move::~Move() {}

bool Move::setTargetPoses(
    const std::string &arm_side, 
    const std::vector<geometry_msgs::Pose> &target_poses)
{
  if (arm_side == "left") {
    left_target_poses_ = target_poses;
  }
  if (arm_side == "right") {
    right_target_poses_ = target_poses;
  }
  joints_mode_ = false;
  return true;
}

bool Move::setTargetPoses(
    const std::string &arm_side, 
    const geometry_msgs::Point &offset)
{
  if (arm_side == "left") {
    left_offset_ = offset;
  }
  if (arm_side == "right") {
    right_offset_ = offset;
  }
  joints_mode_ = false;
  offset_mode_ = true;
  return true;
}

bool Move::setTargetJoints(
    const std::string &arm_side, 
    const std::vector<std::vector<double> > &target_joints) 
{
  if (arm_side == "left") {
    left_target_joints_ = target_joints;
  }
  if (arm_side == "right") {
    right_target_joints_ = target_joints;
  }
  joints_mode_ = true;
  return true;
}

moveit::planning_interface::MoveItErrorCode Move::planJoints(
    const uint16_t &left_start_idx, 
    const uint16_t &right_start_idx)
{ 
  moveit::planning_interface::MoveItErrorCode result(moveit_msgs::MoveItErrorCodes::FAILURE);

  for (uint16_t i = left_start_idx; i <= left_target_joints_.size(); i++) {
    uint16_t right_start = 0;
    if (i == left_start_idx) {
      right_start = right_start_idx;
    }
    for (uint16_t j = right_start; j <= right_target_joints_.size(); j++) {
      if (left_target_joints_.size() > 0 && right_target_joints_.size() > 0 &&
          (i == 0 || j == 0) ) {
        continue;
      }
      
      setTargetToStart();
      bool set_target = false;
      if ( i > 0 && (arm_side_ == "left" || arm_side_ == "both") ) {
        ROS_INFO_STREAM("Trying left target idx " << i << " out of " << left_target_joints_.size());
        arm_controller_->setJointValueTarget(
          listToJoints("left", left_target_joints_[i-1]));
        set_target = true;
      }
      if ( j > 0 && (arm_side_ == "right" || arm_side_ == "both")) {
        ROS_INFO_STREAM("Trying right target idx " << j << " out of " << right_target_joints_.size());
        arm_controller_->setJointValueTarget(
          listToJoints("right", right_target_joints_[j-1]));
        ROS_INFO("Set target");
        set_target = true;
      }
      if (set_target) {
        result = planRetry(*arm_controller_, motion_plan_, retries_);
      }
      if (result) {
        ROS_INFO("Solution found.");
        left_start_idx_ = i;
        right_start_idx_ = j;
        return result;
      }
    }
  } 

  return result;
}

moveit::planning_interface::MoveItErrorCode Move::planPoses(
    const uint16_t &left_start_idx, 
    const uint16_t &right_start_idx)
{
  moveit::planning_interface::MoveItErrorCode result(moveit_msgs::MoveItErrorCodes::FAILURE);

  for (uint16_t i = left_start_idx; i <= left_target_poses_.size(); i++) {
    uint16_t right_start = 0;
    if (i == left_start_idx_) {
      right_start = right_start_idx;
    }
    for (uint16_t j = right_start; j <= right_target_poses_.size(); j++) {
      if (left_target_poses_.size() > 0 && right_target_poses_.size() > 0 &&
          (i == 0 || j == 0) ) {
        continue;
      }

      arm_controller_->clearPoseTargets();
      bool set_target = false;
      if ( i > 0 && (arm_side_ == "left" || arm_side_ == "both") ) {
        ROS_INFO_STREAM("Trying left target idx " << i << " out of " << left_target_poses_.size());
        arm_controller_->setPoseTarget(left_target_poses_[i-1], "left_gripper");
        set_target = true;
      }
      if ( j > 0 && (arm_side_ == "right" || arm_side_ == "both")) {
        ROS_INFO_STREAM("Trying right target idx " << j << " out of " << right_target_poses_.size());
        arm_controller_->setPoseTarget(right_target_poses_[j-1], "right_gripper");
        set_target = true;
      }
      if (set_target) {
        result = planRetry(*arm_controller_, motion_plan_, retries_);
      }
      if (result) {
        ROS_INFO("Solution found.");
        left_start_idx_ = i;
        right_start_idx_ = j;
        return result;
      }
    }
  } 

  return result;
}


G1ControlErrorCode Move::plan()
{
  if (state_.val == g1_control_msgs::G1ControlStateCodes::PLANNED) {
    return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::SUCCESS);
  }

  if (state_.val == g1_control_msgs::G1ControlStateCodes::INITIALIZED) {
    if (arm_controller_ == NULL) {
      return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::CONTROLLER_ERROR);
    }

    if (arm_side_ == "left") {
      right_target_joints_.clear();
      right_target_poses_.clear();
    }
    else if (arm_side_ == "right") {
      left_target_joints_.clear();
      left_target_poses_.clear();
    }
    
    moveit::planning_interface::MoveItErrorCode result(moveit_msgs::MoveItErrorCodes::FAILURE);

    arm_controller_->setStartState(start_state_);
    setTargetToStart();

    arm_controller_->clearPathConstraints();
    setConstraints();
    
    if (allowed_collision_objects_.size() > 0) {
      planning_scene_interface_->addAllowedCollision(allowed_collision_objects_);
    }

    if (joints_mode_) {
      result = planJoints(left_start_idx_, right_start_idx_);
    } 
    else {
      result = planPoses(left_start_idx_, right_start_idx_);
    }

    arm_controller_->clearPathConstraints();
    
    if (allowed_collision_objects_.size() > 0) {
      planning_scene_interface_->removeAllowedCollision(allowed_collision_objects_);
    }

    if (!result) {
      state_.val = g1_control_msgs::G1ControlStateCodes::ERROR;
      return G1ControlErrorCode(result.val);
    }
    else {
      state_.val = g1_control_msgs::G1ControlStateCodes::PLANNED; 
      return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::SUCCESS);
    }

  }

  return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::FAILURE);
}

G1ControlErrorCode Move::run()
{

  G1ControlErrorCode result = plan();

  if (result) {
    moveit::planning_interface::MoveItErrorCode success = arm_controller_->execute(motion_plan_);
    if (success) {
      state_.val = g1_control_msgs::G1ControlStateCodes::EXECUTED;
    }
    else {
      state_.val = g1_control_msgs::G1ControlStateCodes::ERROR;
    }

    return G1ControlErrorCode(success.val); 
  }

  return result;
}

moveit_msgs::RobotState Move::getEndState()
{
  moveit_msgs::RobotState end_state;
  if (state_.val == g1_control_msgs::G1ControlStateCodes::PLANNED) {
    end_state = g1::control::getEndState(motion_plan_);
  }
  return end_state;
}

void Move::setStartState(const moveit_msgs::RobotState &start_state)
{
  Action::setStartState(start_state);
  left_start_idx_ = 0;
  right_start_idx_ = 0;
  motion_plan_ = moveit::planning_interface::MoveGroup::Plan();
  ROS_INFO_STREAM("offset_mode_ is ");

  if (offset_mode_) {
    ROS_INFO_STREAM("Set target poses according to offset");
    // Set left target poses according to offsets.
    if (left_offset_.x != 0 || left_offset_.y != 0 || left_offset_.z != 0) {
      ROS_INFO_STREAM("Set target poses according to offset");
      left_target_poses_.clear();
      geometry_msgs::Pose target_pose = getPosefromRSMsg(*arm_controller_, start_state, "left_gripper");
      target_pose = updatePose(target_pose, left_offset_.x, left_offset_.y, left_offset_.z);
      left_target_poses_.push_back(target_pose);
    }
    if (right_offset_.x != 0 || right_offset_.y != 0 || right_offset_.z != 0) {
      right_target_poses_.clear();
      geometry_msgs::Pose target_pose = getPosefromRSMsg(*arm_controller_, start_state, "right_gripper");
      target_pose = updatePose(target_pose, right_offset_.x, right_offset_.y, right_offset_.z);
      right_target_poses_.push_back(target_pose);
    }
  }
}

bool Move::nextTarget()
{
  bool result = true;
  int left_size = (joints_mode_) ? left_target_joints_.size():left_target_poses_.size();
  int right_size = (joints_mode_) ? right_target_joints_.size():right_target_poses_.size();

  if (left_start_idx_ > left_size) {
    result = false;
  }
  else {
    right_start_idx_++;
    if (right_start_idx_ > right_size) {
      right_start_idx_ = 0;
      left_start_idx_++;
      if (left_start_idx_ > left_size) {
        result = false;
      }
    }
  } 

  motion_plan_ = moveit::planning_interface::MoveGroup::Plan();
  state_.val = g1_control_msgs::G1ControlStateCodes::INITIALIZED;
  return result;
}

// Grasp Action
Grasp::Grasp(const std::string &name)
  : Action(name, "Grasp"),
    left_command_(0),
    right_command_(0),
    left_grasp_type_(0),
    right_grasp_type_(0)
{
}

moveit_msgs::RobotState Grasp::getEndState()
{
  return start_state_;
}

void Grasp::setGraspCmd(const std::string &side, const int &cmd, const int &grasp_type)
{
  if (side == "left") {
    left_command_ = cmd;
    left_grasp_type_ = grasp_type;
  }
  else if (side == "right") {
    right_command_ = cmd;
    right_grasp_type_ = grasp_type;
  }
}

G1ControlErrorCode Grasp::plan() 
{
  return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::SUCCESS);
}

G1ControlErrorCode Grasp::run() 
{
  G1ControlErrorCode result(g1_control_msgs::G1ControlErrorCodes::FAILURE);
  if (left_hand_controller_) {
    switch (left_command_) {
      case g1_control_msgs::G1ControlGraspCodes::RESET:
        result = left_hand_controller_->reset();
        break;
      case g1_control_msgs::G1ControlGraspCodes::OPEN:
        result = left_hand_controller_->open();
        break;
      case g1_control_msgs::G1ControlGraspCodes::PREOPEN:
        result = left_hand_controller_->preOpen(left_grasp_type_);
        break;
      case g1_control_msgs::G1ControlGraspCodes::CLOSE:
        result = left_hand_controller_->close(left_grasp_type_);
        break;
      case g1_control_msgs::G1ControlGraspCodes::PREGRASP:
        result = left_hand_controller_->preGrasp(left_grasp_type_);
        break;
      case g1_control_msgs::G1ControlGraspCodes::CALIBRATE:
        result = left_hand_controller_->calibrate();
        break;
      default:
        break;
    };
  }
  if (right_hand_controller_) {
    switch (right_command_) {
      case g1_control_msgs::G1ControlGraspCodes::RESET:
        result = right_hand_controller_->reset();
        break;
      case g1_control_msgs::G1ControlGraspCodes::OPEN:
        result = right_hand_controller_->open();
        break;
      case g1_control_msgs::G1ControlGraspCodes::PREOPEN:
        result = right_hand_controller_->preOpen(left_grasp_type_);
        break;
      case g1_control_msgs::G1ControlGraspCodes::CLOSE:
        result = right_hand_controller_->close(left_grasp_type_);
        break;
      case g1_control_msgs::G1ControlGraspCodes::PREGRASP:
        result = right_hand_controller_->preGrasp(right_grasp_type_);
        break;
      case g1_control_msgs::G1ControlGraspCodes::CALIBRATE:
        result = right_hand_controller_->calibrate();
        break;
      default:
        break;
    };
  }

  return result;
}

// Pick Action
Pick::Pick(const std::string &name)
  : Action(name, "Pick"),
    start_idx_(1),
    solution_idx_(0),
    arm_use_("unknown")
{
}

Pick::~Pick() {}

bool Pick::setTargetPicks(
      const std::string &arm_use, 
      const world_model_msgs::Object &object,
      const std::vector<geometry_msgs::Pose> &approach_poses,
      const std::vector<geometry_msgs::Pose> &grasp_poses,
      const std::vector<int> &grasp_types)
{
  arm_use_ = arm_use;
  object_ = object;
  approach_poses_ = approach_poses;
  grasp_poses_ = grasp_poses;
  grasp_types_ = grasp_types;
  return true;
}

moveit_msgs::RobotState Pick::getEndState()
{
  moveit_msgs::RobotState end_state;
  if (state_.val == g1_control_msgs::G1ControlStateCodes::PLANNED) {
    end_state = g1::control::getEndState(grasp_plan_);
  }
  return end_state;
}

void Pick::setStartState(const moveit_msgs::RobotState &start_state)
{
  Action::setStartState(start_state);
  start_idx_ = 1;
  solution_idx_ = 0;
  approach_plan_ = moveit::planning_interface::MoveGroup::Plan();
  grasp_plan_ = moveit::planning_interface::MoveGroup::Plan();
}

bool Pick::nextTarget()
{
  bool result = true;

  start_idx_++;
  if (start_idx_ > approach_poses_.size()) {
    result = false;
  } 
  solution_idx_ = 0;
  approach_plan_ = moveit::planning_interface::MoveGroup::Plan();
  grasp_plan_ = moveit::planning_interface::MoveGroup::Plan();
  state_.val = g1_control_msgs::G1ControlStateCodes::INITIALIZED;

  return result;
}

int Pick::getGraspType()
{
  if (state_.val == g1_control_msgs::G1ControlStateCodes::PLANNED && solution_idx_ > 0) {
    return grasp_types_[solution_idx_-1];
  }
  ROS_ERROR_STREAM("Cannot get grasp type as no solution found.");
  return -1;
}

moveit::planning_interface::MoveItErrorCode Pick::planPick(const int &start_idx)
{
  moveit::planning_interface::MoveItErrorCode result(moveit_msgs::MoveItErrorCodes::FAILURE);
  const std::string end_effector = (arm_use_ == "left") ? "left_gripper" : "right_gripper";

  for (uint16_t i = start_idx; i <= approach_poses_.size(); i++) {
    // Plan approach
    ROS_INFO_STREAM("Trying approach pose idx " << i << " out of " << approach_poses_.size());
    arm_controller_->setStartState(start_state_);
    setTargetToStart();

    // setLockOrientation(arm_use_, grasp_types_[i-1]);
    // arm_controller_->clearPathConstraints();
    // setConstraints();

    arm_controller_->clearPoseTargets();
    arm_controller_->setPoseTarget(approach_poses_[i-1], end_effector);
    result = planRetry(*arm_controller_, approach_plan_, retries_);
    if (!result) {
      continue;
    }
    
    if (allowed_collision_objects_.size() > 0) {
      planning_scene_interface_->addAllowedCollision(allowed_collision_objects_);
    }

    // Plan grasp
    ROS_INFO_STREAM("Trying grasp pose idx " << i << " out of " << grasp_poses_.size());
    arm_controller_->setStartState(g1::control::getEndState(approach_plan_));
    setTargetToStart();

    // setLockPosition(arm_use_, 0.05, 0.05, 1);
    // setLockOrientation(arm_use_, grasp_types_[i-1]);
    arm_controller_->clearPathConstraints();
    setConstraints();

    arm_controller_->clearPoseTargets();
    arm_controller_->setPoseTarget(grasp_poses_[i-1], end_effector);
    result = planRetry(*arm_controller_, grasp_plan_, retries_);

    arm_controller_->clearPathConstraints();

    if (allowed_collision_objects_.size() > 0) {
      planning_scene_interface_->removeAllowedCollision(allowed_collision_objects_);
    }

    if (result) {
      ROS_INFO("Solution found.");
      solution_idx_ = i;
      start_idx_ = i;
      return result;
    }
  } 

  return result;
}

G1ControlErrorCode Pick::plan()
{
  if (state_.val == g1_control_msgs::G1ControlStateCodes::PLANNED) {
    return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::SUCCESS);
  }

  if (state_.val == g1_control_msgs::G1ControlStateCodes::INITIALIZED) {
    if (arm_controller_ == NULL) {
      return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::CONTROLLER_ERROR);
    }

    if (arm_side_ != "both" && arm_side_ != arm_use_) {
      ROS_ERROR_STREAM("Cannot use " << arm_use_ << " arm.");
      return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::CONTROLLER_ERROR);
    }
    
    moveit::planning_interface::MoveItErrorCode result(moveit_msgs::MoveItErrorCodes::FAILURE);

    result = planPick(start_idx_);

    if (!result) {
      state_.val = g1_control_msgs::G1ControlStateCodes::ERROR;
      return G1ControlErrorCode(result.val);
    }
    else {
      state_.val = g1_control_msgs::G1ControlStateCodes::PLANNED; 
      return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::SUCCESS);
    }

  }

  return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::FAILURE);
}

G1ControlErrorCode Pick::run()
{

  G1ControlErrorCode result = plan();
  state_.val = g1_control_msgs::G1ControlStateCodes::ERROR;

  if (result) {
    //execute approach
    moveit::planning_interface::MoveItErrorCode success = arm_controller_->execute(approach_plan_);
    result.val = success.val;
    if (!result) {
      return result;
    }
    ros::Duration(0.5).sleep();

    //execute preGrasp
    HandController *hand_controller = (arm_use_ == "left") ? left_hand_controller_: right_hand_controller_;
    result = hand_controller->preGrasp(grasp_types_[solution_idx_-1]);
    if (!result) {
      return result;
    }
    ros::Duration(0.5).sleep();

    //execute grasp plan
    success = arm_controller_->execute(grasp_plan_);
    result.val = success.val;
    if (!result) {
      return result;
    }
    ros::Duration(0.5).sleep();

    //execute grasp
    result = hand_controller->close(grasp_types_[solution_idx_-1]);
    if (!result) {
      return result;
    }

    //Attach the object to end effector
    std::string end_effector = (arm_use_ == "left") ? "left_gripper" : "right_gripper";
    arm_controller_->attachObject(object_.id, end_effector);

    state_.val = g1_control_msgs::G1ControlStateCodes::EXECUTED;
    return G1ControlErrorCode(result); 
  }

  return result;
}

// Place Action
Place::Place(const std::string &name)
  : Action(name, "Place"),
    start_idx_(1),
    arm_use_("unknown"),
    grasp_type_(0)
{
}

Place::~Place() {}

bool Place::setTargetPlaces(
      const std::string &arm_use, 
      const world_model_msgs::Object &object,
      const std::vector<geometry_msgs::Pose> &place_poses,
      const std::vector<geometry_msgs::Pose> &retreat_poses,
      const int &grasp_type,
      const geometry_msgs::Point &preplace_offset)
{
  arm_use_ = arm_use;
  object_ = object;
  preplace_offset_ = preplace_offset; 
  place_poses_ = place_poses;
  retreat_poses_ = retreat_poses;
  grasp_type_ = grasp_type;
  return true;
}

moveit_msgs::RobotState Place::getEndState()
{
  moveit_msgs::RobotState end_state;
  if (state_.val == g1_control_msgs::G1ControlStateCodes::PLANNED) {
    end_state = g1::control::getEndState(retreat_plan_);
  }
  return end_state;
}

void Place::setStartState(const moveit_msgs::RobotState &start_state)
{
  Action::setStartState(start_state);
  start_idx_ = 1;
  preplace_plan_ = moveit::planning_interface::MoveGroup::Plan();
  place_plan_ = moveit::planning_interface::MoveGroup::Plan();
  retreat_plan_ = moveit::planning_interface::MoveGroup::Plan();
}

bool Place::nextTarget()
{
  bool result = true;

  start_idx_++;
  if (start_idx_ > place_poses_.size()) {
    result = false;
  } 
  preplace_plan_ = moveit::planning_interface::MoveGroup::Plan();
  place_plan_ = moveit::planning_interface::MoveGroup::Plan();
  retreat_plan_ = moveit::planning_interface::MoveGroup::Plan();
  state_.val = g1_control_msgs::G1ControlStateCodes::INITIALIZED;

  return result;
}

moveit::planning_interface::MoveItErrorCode Place::planPlace(const int &start_idx)
{
  moveit::planning_interface::MoveItErrorCode result(moveit_msgs::MoveItErrorCodes::FAILURE);
  const std::string end_effector = (arm_use_ == "left") ? "left_gripper" : "right_gripper";
  
  geometry_msgs::Pose preplace_pose;

  for (uint16_t i = start_idx; i <= place_poses_.size(); i++) {
    if (allowed_collision_objects_.size() > 0) {
      planning_scene_interface_->addAllowedCollision(allowed_collision_objects_);
    }

    // Plan preplace
    ROS_INFO_STREAM("Trying preplace pose idx " << i << " out of " << place_poses_.size());
    arm_controller_->setStartState(start_state_);
    setTargetToStart();

    setLockOrientation(arm_use_, grasp_type_);
    arm_controller_->clearPathConstraints();
    setConstraints();

    preplace_pose = g1::control::updatePose(place_poses_[i-1], preplace_offset_.x, preplace_offset_.y, preplace_offset_.z); 
    
    arm_controller_->clearPoseTargets();
    arm_controller_->setPoseTarget(preplace_pose, end_effector);
    result = planRetry(*arm_controller_, preplace_plan_, retries_);
    if (!result) {
      continue;
    }

    // Plan place
    ROS_INFO_STREAM("Trying place pose idx " << i << " out of " << place_poses_.size());
    arm_controller_->setStartState(g1::control::getEndState(preplace_plan_));
    setTargetToStart();

    setLockOrientation(arm_use_, grasp_type_);
    arm_controller_->clearPathConstraints();
    setConstraints();

    arm_controller_->clearPoseTargets();
    arm_controller_->setPoseTarget(place_poses_[i-1], end_effector);
    result = planRetry(*arm_controller_, place_plan_, retries_);
    if (!result) {
      continue;
    }
  
    // Plan retreat
    ROS_INFO_STREAM("Trying retreat pose idx " << i << " out of " << retreat_poses_.size());
    arm_controller_->setStartState(g1::control::getEndState(place_plan_));
    setTargetToStart();

    setLockOrientation(arm_use_, grasp_type_);
    arm_controller_->clearPathConstraints();
    setConstraints();

    arm_controller_->clearPoseTargets();
    arm_controller_->setPoseTarget(retreat_poses_[i-1], end_effector);
    result = planRetry(*arm_controller_, retreat_plan_, retries_);

    arm_controller_->clearPathConstraints();

    if (allowed_collision_objects_.size() > 0) {
      planning_scene_interface_->removeAllowedCollision(allowed_collision_objects_);
    }

    if (result) {
      ROS_INFO("Solution found.");
      start_idx_ = i;
      return result;
    }
  } 

  return result;
}

G1ControlErrorCode Place::plan()
{
  if (state_.val == g1_control_msgs::G1ControlStateCodes::PLANNED) {
    return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::SUCCESS);
  }

  if (state_.val == g1_control_msgs::G1ControlStateCodes::INITIALIZED) {
    if (arm_controller_ == NULL) {
      return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::CONTROLLER_ERROR);
    }

    if (arm_side_ != "both" && arm_side_ != arm_use_) {
      ROS_ERROR_STREAM("Cannot use " << arm_use_ << " arm.");
      return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::CONTROLLER_ERROR);
    }
    
    moveit::planning_interface::MoveItErrorCode result(moveit_msgs::MoveItErrorCodes::FAILURE);

    result = planPlace(start_idx_);

    if (!result) {
      state_.val = g1_control_msgs::G1ControlStateCodes::ERROR;
      return G1ControlErrorCode(result.val);
    }
    else {
      state_.val = g1_control_msgs::G1ControlStateCodes::PLANNED; 
      return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::SUCCESS);
    }

  }

  return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::FAILURE);
}

G1ControlErrorCode Place::run()
{

  G1ControlErrorCode result = plan();
  state_.val = g1_control_msgs::G1ControlStateCodes::ERROR;

  if (result) {
    //execute preplace
    moveit::planning_interface::MoveItErrorCode success = arm_controller_->execute(preplace_plan_);
    result.val = success.val;
    if (!result) {
      return result;
    }
    ros::Duration(0.5).sleep();

    //execute place
    success = arm_controller_->execute(place_plan_);
    result.val = success.val;
    if (!result) {
      return result;
    }
    ros::Duration(0.5).sleep();

    //execute preOpen 
    HandController *hand_controller = (arm_use_ == "left") ? left_hand_controller_: right_hand_controller_;
    
    if (hand_controller->getType() == "reflex") {
      result = hand_controller->preOpen(grasp_type_);
    }
    else {
      result = hand_controller->open();
    }
    
    if (!result) {
      return result;
    }
    ros::Duration(0.5).sleep();

    //Detach object
    arm_controller_->detachObject(object_.id);

    //execute retreat
    success = arm_controller_->execute(retreat_plan_);
    result.val = success.val;
    if (!result) {
      return result;
    }
    ros::Duration(0.5).sleep();

    //execute open
    if (hand_controller->getType() == "reflex") {
      result = hand_controller->open();
      if (!result) {
        return result;
      }
    }

    state_.val = g1_control_msgs::G1ControlStateCodes::EXECUTED;
    return G1ControlErrorCode(result); 
  }

  return result;
}



}
}

