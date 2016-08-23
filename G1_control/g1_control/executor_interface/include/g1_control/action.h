#ifndef G1_CONTROL_ACTION_
#define G1_CONTROL_ACTION_

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/conversions.h>
#include <g1_control_msgs/G1ControlErrorCodes.h>
#include <g1_control_msgs/G1ControlStateCodes.h>
#include <g1_control_msgs/error_code.h>
#include <g1_control_msgs/G1ControlGraspCodes.h>
#include <g1_control/utilities.h>
#include <g1_control/hand_controller.h>
#include <g1_control/planning_scene_interface.h>
#include <world_model_msgs/Object.h>
#include <boost/shared_ptr.hpp>

namespace g1
{

namespace control
{

class Action
{
public:
  
  Action(const std::string &name = std::string("Empty"), const std::string &type = std::string("base"))
    : name_(name),
      type_(type),
      retries_(1),
      state_(G1ControlState(g1_control_msgs::G1ControlStateCodes::INITIALIZED)),
      arm_controller_(NULL),
      left_hand_controller_(NULL),
      right_hand_controller_(NULL),
      planning_scene_interface_(NULL)
  {}

  ~Action(){};

  virtual G1ControlErrorCode plan() = 0;

  virtual G1ControlErrorCode run() = 0;

  virtual moveit_msgs::RobotState getEndState() = 0;

  virtual void setStartState(const moveit_msgs::RobotState &start_state)
  {
    start_state_ = start_state;
    state_.val = g1_control_msgs::G1ControlStateCodes::INITIALIZED;
  }

  virtual bool nextTarget()
  {
    return false;
  }

  void setRetries(const int &retries)
  {
    retries_ = retries; 
  }

  const std::string& getName() const
  {
    return name_;
  }

  const std::string& getType() const
  {
    return type_;
  }

  void setName(const std::string &name)
  {
    name_ = name;
  }

  void setArmController(
      moveit::planning_interface::MoveGroup *arm_controller, 
      const std::string &arm_side)
  {
    arm_controller_ = arm_controller;
    arm_side_ = arm_side;
  }

  void setHandController(
      HandController *left_hand_controller,
      HandController *right_hand_controller)
  {
    left_hand_controller_ = left_hand_controller;
    right_hand_controller_ = right_hand_controller;
  }

  void setPlanningScene(
      PlanningSceneInterface *planning_scene_interface)
  {
    planning_scene_interface_ = planning_scene_interface;
  }

  void setLockOrientation(
    const std::string &arm_side,
    const double &x_tolerance = 0.1, 
    const double &y_tolerance = 0.1, 
    const double &z_tolerance = 0.1)
  {
    moveit_msgs::OrientationConstraint ocm;
    ocm.absolute_x_axis_tolerance = x_tolerance;
    ocm.absolute_y_axis_tolerance = y_tolerance;
    ocm.absolute_z_axis_tolerance = z_tolerance;
    ocm.weight = 1.0;
    ocm.link_name = (arm_side == "left") ? "left_gripper": "right_gripper";

    constraints_.orientation_constraints.clear();
    constraints_.orientation_constraints.push_back(ocm);
  }

  void setLockOrientation(
    const std::string &arm_side,
    const int &grasp_type)
  {
    if (grasp_type == 0 || grasp_type == 4) { 
      setLockOrientation(arm_side, 0.1, 1000, 0.1);
    }
    else if (grasp_type == 1 || grasp_type == 2) {
      setLockOrientation(arm_side, 1000, 0.1, 0.1);
    } 
    else if (grasp_type == 3) {
      setLockOrientation(arm_side, 0.1, 0.1, 1000);
    }
  }

  void setLockPosition(
    const std::string &arm_side,
    const double &x_dimension = 0.02, 
    const double &y_dimension = 0.02, 
    const double &z_dimension = 0.02)
  {
    moveit_msgs::PositionConstraint pcm;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = x_dimension;
    primitive.dimensions[1] = y_dimension;
    primitive.dimensions[2] = z_dimension;
    pcm.constraint_region.primitives.push_back(primitive);
    pcm.weight = 1.0;
    pcm.link_name = (arm_side == "left") ? "left_gripper": "right_gripper";

    constraints_.position_constraints.clear();
    constraints_.position_constraints.push_back(pcm);
  }

  void setConstraints()
  {
    for (int i = 0; i < constraints_.position_constraints.size(); i++) {
      std::string end_effector = constraints_.position_constraints[i].link_name;
      geometry_msgs::Pose start_pose = getPosefromRSMsg(*arm_controller_, start_state_, end_effector); 

      constraints_.position_constraints[i].header.frame_id = "base";
      constraints_.position_constraints[i].constraint_region.primitive_poses.push_back(start_pose);
    }

    for (int i = 0; i < constraints_.orientation_constraints.size(); i++) {
      std::string end_effector = constraints_.orientation_constraints[i].link_name;
      geometry_msgs::Pose start_pose = getPosefromRSMsg(*arm_controller_, start_state_, end_effector); 

      constraints_.orientation_constraints[i].header.frame_id = "base";
      constraints_.orientation_constraints[i].orientation = start_pose.orientation;
    }

    if (constraints_.position_constraints.size() > 0 || constraints_.orientation_constraints.size() > 0) {
      arm_controller_->setPathConstraints(constraints_);
    }

    ROS_INFO("Set Constraints");
  }

  void setAllowedCollision(const std::vector<std::string> &object_ids)
  {
    allowed_collision_objects_ = object_ids;
  }

  void setTargetToStart()
  {
    moveit_msgs::RobotState robot_state = start_state_;
    moveit::core::RobotState state(*(arm_controller_->getCurrentState()));
    moveit::core::robotStateMsgToRobotState(robot_state, state);
    arm_controller_->setJointValueTarget(state);
  }  

protected:
  std::string name_;
  std::string type_;
  int retries_;
  G1ControlState state_;
  moveit_msgs::RobotState start_state_;

  std::string arm_side_;
  moveit::planning_interface::MoveGroup *arm_controller_;
  PlanningSceneInterface *planning_scene_interface_;
  HandController *left_hand_controller_, *right_hand_controller_;

  std::vector<std::string> allowed_collision_objects_;
  moveit_msgs::Constraints constraints_;
};

class Move: public Action
{
public:
  Move(const std::string &name);

  ~Move();

  bool setTargetPoses(
      const std::string &arm_side, 
      const std::vector<geometry_msgs::Pose> &target_poses);

  bool setTargetPoses(
      const std::string &arm_side, 
      const geometry_msgs::Point &offset);

  bool setTargetJoints(
      const std::string &arm_side, 
      const std::vector<std::vector<double> > &target_joints);

  G1ControlErrorCode plan();

  G1ControlErrorCode run();

  moveit_msgs::RobotState getEndState();

  void setStartState(const moveit_msgs::RobotState &start_state);

  bool nextTarget();

private:

  moveit::planning_interface::MoveItErrorCode planJoints(
      const uint16_t &left_start_idx,
      const uint16_t &right_start_idx);
  moveit::planning_interface::MoveItErrorCode planPoses(
      const uint16_t &left_start_idx,
      const uint16_t &right_start_idx);

  std::vector<geometry_msgs::Pose> left_target_poses_;
  std::vector<geometry_msgs::Pose> right_target_poses_;

  std::vector<std::vector<double> > left_target_joints_;
  std::vector<std::vector<double> > right_target_joints_;

  bool joints_mode_;
  bool offset_mode_;
  geometry_msgs::Point left_offset_, right_offset_;

  uint16_t left_start_idx_;
  uint16_t right_start_idx_;

  moveit::planning_interface::MoveGroup::Plan motion_plan_;
};

class Grasp: public Action
{
public:
  Grasp(const std::string &name);

  ~Grasp() {};

  void setGraspCmd(const std::string &side, const int &cmd, const int &grasp_type);

  G1ControlErrorCode plan();

  G1ControlErrorCode run();

  moveit_msgs::RobotState getEndState();

private:
  int left_command_;
  int right_command_;
  int left_grasp_type_;
  int right_grasp_type_;

};

class Pick: public Action
{
public:
  Pick(const std::string &name);

  ~Pick();

  bool setTargetPicks(
      const std::string &arm_use, 
      const world_model_msgs::Object &object,
      const std::vector<geometry_msgs::Pose> &approach_poses,
      const std::vector<geometry_msgs::Pose> &grasp_poses,
      const std::vector<int> &grasp_types);

  G1ControlErrorCode plan();

  G1ControlErrorCode run();

  moveit_msgs::RobotState getEndState();

  void setStartState(const moveit_msgs::RobotState &start_state);

  bool nextTarget();

  int getGraspType();

private:
  moveit::planning_interface::MoveItErrorCode planPick(const int &start_idx);

  std::vector<geometry_msgs::Pose> approach_poses_;
  std::vector<geometry_msgs::Pose> grasp_poses_;

  uint16_t start_idx_, solution_idx_;
  std::string arm_use_;
  world_model_msgs::Object object_;
  std::vector<int> grasp_types_;

  moveit::planning_interface::MoveGroup::Plan approach_plan_;
  moveit::planning_interface::MoveGroup::Plan grasp_plan_;
};

class Place: public Action
{
public:
  Place(const std::string &name);

  ~Place();

  bool setTargetPlaces(
      const std::string &arm_use, 
      const world_model_msgs::Object &object,
      const std::vector<geometry_msgs::Pose> &place_poses,
      const std::vector<geometry_msgs::Pose> &retreat_poses,
      const int &grasp_type,
      const geometry_msgs::Point &preplace_offset = g1::control::constructPoint(0, 0, 0));

  G1ControlErrorCode plan();

  G1ControlErrorCode run();

  moveit_msgs::RobotState getEndState();

  void setStartState(const moveit_msgs::RobotState &start_state);

  bool nextTarget();

private:
  moveit::planning_interface::MoveItErrorCode planPlace(const int &start_idx);

  std::vector<geometry_msgs::Pose> place_poses_, retreat_poses_;

  uint16_t start_idx_;
  std::string arm_use_;
  world_model_msgs::Object object_;
  int grasp_type_;
  geometry_msgs::Point preplace_offset_;

  moveit::planning_interface::MoveGroup::Plan preplace_plan_, place_plan_, retreat_plan_;
};

typedef boost::shared_ptr<Action> ActionPtr;
typedef boost::shared_ptr<Move> MovePtr;
typedef boost::shared_ptr<Grasp> GraspPtr;
typedef boost::shared_ptr<Pick> PickPtr;
typedef boost::shared_ptr<Place> PlacePtr;
}
}

#endif

