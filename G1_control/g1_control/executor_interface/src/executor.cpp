/*
 * =====================================================================================
 *
 *       Filename:  executor.cpp
 *
 *    Description:  Library for baxter execution
 *
 *        Version:  1.0
 *        Created:  06/14/2016 04:06:13 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ren Mao (neroam), neroam@umd.edu
 *   Organization:  
 *
 * =====================================================================================
 */

#include <stdexcept>
#include <ros/console.h>
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/CollisionObject.h>

#include <g1_control/executor.h>
#include <g1_control/hand_controller.h>
#include <g1_control/utilities.h>
#include <g1_control/planning_scene_interface.h>

#include <actionlib/client/simple_action_client.h>
#include <grasp_generator/GenerateGraspsAction.h>

namespace g1
{

namespace control
{

class Executor::ExecutorImpl
{
public:

  ExecutorImpl(const Options &opt)
    : opt_(opt),
      node_handle_(opt.node_handle_),
      arms_controller_(NULL),
      left_hand_controller_(NULL),
      right_hand_controller_(NULL),
      state_(g1_control_msgs::G1ControlStateCodes::INITIALIZED),
      reflex_planner_("grasp_generator_server/reflex")
  {
    std::string arm = opt_.arm_side_ + "_arm";
    if (opt_.arm_side_ == "both") {
      arm = "both_arms";
    }
    arms_controller_ = new moveit::planning_interface::MoveGroup(arm);
    if (!arms_controller_) {
      ROS_ERROR("Cannot initialize executor.");
    }
    arms_controller_->setPlannerId("RRTConnectkConfigDefault");

    planning_scene_interface_ = new PlanningSceneInterface();

    // Hand Controller
    if (opt_.arm_side_ == "left" || opt_.arm_side_ == "both") 
    {
      if (opt_.left_hand_type_ == "reflex") {
        left_hand_controller_ = new ReflexHandController(node_handle_) ;
      }
      else if (opt_.left_hand_type_ == "electric") {
        left_hand_controller_ = new BaxterGripperController(node_handle_, "left");
      }
    }
    if (opt_.arm_side_ == "right" || opt_.arm_side_ == "both")
    {
      if (opt_.right_hand_type_ == "reflex") {
        right_hand_controller_ = new ReflexHandController(node_handle_) ;
      }
      else if (opt_.right_hand_type_ == "electric") {
        right_hand_controller_ = new BaxterGripperController(node_handle_, "right");
      }
    }

    if ( !node_handle_.getParam("/g1_control/initial_arms/left", init_arms_joints_["left"]) ||
         !node_handle_.getParam("/g1_control/initial_arms/right", init_arms_joints_["right"])) {
      ROS_ERROR("Failed to load initial arms parameters.");
    }

    if ( !node_handle_.getParam("/g1_control/initial_arms/grasp/0", init_arms_joints_grasp_[0]) || 
         !node_handle_.getParam("/g1_control/initial_arms/grasp/1", init_arms_joints_grasp_[1]) || 
         !node_handle_.getParam("/g1_control/initial_arms/grasp/2", init_arms_joints_grasp_[2]) ||
         !node_handle_.getParam("/g1_control/initial_arms/grasp/3", init_arms_joints_grasp_[3]) ||
         !node_handle_.getParam("/g1_control/initial_arms/grasp/4", init_arms_joints_grasp_[4])) {
      ROS_ERROR("Failed to load initial arms parameters for grasping poses.");
    }

    ROS_INFO("Reference frame: %s", arms_controller_->getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", arms_controller_->getEndEffectorLink().c_str());
    ROS_INFO("Executor for arms %s initialized.", opt_.arm_side_.c_str());

//     ROS_INFO_STREAM(arms_controller_->getEndEffectorLink());
//     ROS_INFO_STREAM(arms_controller_->getEndEffector());

    initialize();
  }

  ~ExecutorImpl()
  {
    if (arms_controller_) {
      delete arms_controller_;
    }
    if (left_hand_controller_) {
      delete left_hand_controller_;
    }
    if (right_hand_controller_) {
      delete right_hand_controller_;
    }
    if (planning_scene_interface_) {
      delete planning_scene_interface_;
    }
  }

  void initialize()
  {
    action_goals_.clear();
    state_.val = g1_control_msgs::G1ControlStateCodes::INITIALIZED;
  }

  G1ControlErrorCode resetArms(const std::string &side)
  {
    initialize();
    boost::shared_ptr<Move> move_ptr(new Move("Reset Arms"));
    if (side == "left" || side == "both") {
      move_ptr->setTargetJoints("left", 
        std::vector<std::vector<double> > (1, init_arms_joints_["left"]));
    }
    if (side == "right" || side == "both") {
      move_ptr->setTargetJoints("right", 
        std::vector<std::vector<double> > (1, init_arms_joints_["right"]));
    }
    addActionTarget(move_ptr);
    return run();
  }

  G1ControlErrorCode calibrateGrippers(const std::string &side)
  {
    initialize();
    boost::shared_ptr<Grasp> grasp_ptr(new Grasp("Calibrate Grippers"));
    if (side == "left" || side == "both") {
      grasp_ptr->setGraspCmd("left", 
          g1_control_msgs::G1ControlGraspCodes::CALIBRATE,
          0);
    }
    if (side == "right" || side == "both") {
      grasp_ptr->setGraspCmd("right", 
          g1_control_msgs::G1ControlGraspCodes::CALIBRATE,
          0);
    }
    addActionTarget(grasp_ptr);
    return run();
  }

  G1ControlErrorCode resetGrippers(const std::string &side)
  {
    initialize();
    boost::shared_ptr<Grasp> grasp_ptr(new Grasp("Reset Grippers"));
    if (side == "left" || side == "both") {
      grasp_ptr->setGraspCmd("left", 
          g1_control_msgs::G1ControlGraspCodes::RESET,
          0);
    }
    if (side == "right" || side == "both") {
      grasp_ptr->setGraspCmd("right", 
          g1_control_msgs::G1ControlGraspCodes::RESET,
          0);
    }
    addActionTarget(grasp_ptr);
    return run();
  }

  G1ControlErrorCode initGrippers(const std::string &side)
  {
    initialize();
    boost::shared_ptr<Grasp> grasp_ptr(new Grasp("Init Grippers"));
    if (side == "left" || side == "both") {
      grasp_ptr->setGraspCmd("left", 
          g1_control_msgs::G1ControlGraspCodes::OPEN,
          0);
    }
    if (side == "right" || side == "both") {
      grasp_ptr->setGraspCmd("right", 
          g1_control_msgs::G1ControlGraspCodes::OPEN,
          0);
    }
    addActionTarget(grasp_ptr);
    return run();
  }

  bool addActionTarget(ActionPtr action_goal)
  {
    action_goals_.push_back(action_goal);
    return true;
  }

  G1ControlErrorCode planRecursion(const int &cur_step, const moveit_msgs::RobotState &start_state) {
    if (cur_step >= action_goals_.size()) {
      return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::SUCCESS);
    }

    ActionPtr cur_ptr = action_goals_[cur_step];

    cur_ptr->setArmController(arms_controller_, opt_.arm_side_);
    cur_ptr->setPlanningScene(planning_scene_interface_);
    cur_ptr->setHandController(left_hand_controller_, 
                               right_hand_controller_);
    cur_ptr->setStartState(start_state);
    

    G1ControlErrorCode result(g1_control_msgs::G1ControlErrorCodes::FAILURE);
    do {
        ROS_INFO("#Planning the action goal %d : Action %s : %s.", cur_step+1, 
            cur_ptr->getType().c_str(),
            cur_ptr->getName().c_str());
        result = cur_ptr->plan();

        if (!result) {
          ROS_ERROR("#Planning failed with error code %d.", result.val);
          break;
        } 
        else {
          ROS_INFO("#Planning succeed for action %d.", cur_step+1);
          //Recusive call to plan rest actions
          result = planRecursion(cur_step+1, cur_ptr->getEndState());

          if (result) {
            break;
          } 
        }
    }
    while (cur_ptr->nextTarget());
    return result;
  }

  G1ControlErrorCode plan()
  {
    G1ControlErrorCode result(g1_control_msgs::G1ControlErrorCodes::FAILURE);

    if (action_goals_.size() < 1) {
      ROS_WARN("No action target is set. Abort planning.");
    }

    if (state_.val == g1_control_msgs::G1ControlStateCodes::PLANNED) {
      result.val = g1_control_msgs::G1ControlErrorCodes::SUCCESS;
    } 

    if (state_.val == g1_control_msgs::G1ControlStateCodes::INITIALIZED) {
      moveit_msgs::RobotState start_state;
      moveit::core::RobotStatePtr rs = arms_controller_->getCurrentState();
      moveit::core::robotStateToRobotStateMsg(*rs, start_state);

      result = planRecursion(0, start_state);
      if (!result) {
        state_.val = g1_control_msgs::G1ControlStateCodes::ERROR;
      }
      else {
        ROS_INFO("#All steps planned successfully!");
        state_.val = g1_control_msgs::G1ControlStateCodes::PLANNED;
      }
    }
    return result;
  }

  G1ControlErrorCode run()
  {
    G1ControlErrorCode result = plan();

    if (result) {
      for (int i = 0; i < action_goals_.size(); i++) {
        ROS_INFO("#Executing the action goal %d : Action %s : %s.", i+1, 
            action_goals_[i]->getType().c_str(),
            action_goals_[i]->getName().c_str());

        result = action_goals_[i]->run();
        if (!result) {
          ROS_ERROR("#Execution failed with error code %d.", result.val);
          return result;
        }
        else
        {
          ROS_INFO("#Execution succeed.");
          // ros::Duration(1).sleep();
        }
      }
      action_goals_.clear();
      state_.val = g1_control_msgs::G1ControlStateCodes::INITIALIZED;
      return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::SUCCESS);
    }

    return result;
  }

  geometry_msgs::PoseStamped getCurrentPose(const std::string &end_effector_link)
  {
    return arms_controller_->getCurrentPose(end_effector_link);
  }

  void clearPlanningScene()
  {
    ROS_INFO("Clear planning scene");
    arms_controller_->detachObject();
    planning_scene_interface_->removeCollisionObjects(
        planning_scene_interface_->getKnownObjectNames()
        );
  }

  void updatePlanningScene(const std::vector<world_model_msgs::Object> &objects)
  {
//     clearPlanningScene();

    std::vector<moveit_msgs::CollisionObject> collision_objects;  
    for (int i = 0; i < objects.size(); i++) {
      ROS_INFO_STREAM("Add object into model: " << objects[i].id);
      collision_objects.push_back(
          toCollisionObject(*arms_controller_, objects[i]));
    }
    planning_scene_interface_->addCollisionObjects(collision_objects);
  }

  G1ControlErrorCode openMicrowave(const world_model_msgs::Object &object)
  {

    ROS_INFO_STREAM("Use left hand to open microwave");
    std::string arm_side = "left";
    // resetArms(arm_side);

    geometry_msgs::Pose target_pose;
    std::vector<geometry_msgs::Pose> target_poses;
    std::vector<std::string> collision_objects;

    geometry_msgs::Pose object_pose = object.primitive_poses[0];
    Eigen::Quaternionf object_quat(object_pose.orientation.w,
      object_pose.orientation.x,
      object_pose.orientation.y,
      object_pose.orientation.z);
    Eigen::Matrix3f rotation = object_quat.matrix();
    Eigen::Vector3f x_dir = -1.0 * rotation.col(0);
    Eigen::Vector3f y_dir = -1.0 * rotation.col(1);
    Eigen::Vector3f z_dir = rotation.col(2);
    Eigen::Vector3f offset_vector;


    // Pick the handle
    PickPtr pick_ptr(new Pick("Grasping microwave handle")); 
    offset_vector = -0.1 * x_dir + 0.01 * y_dir;
    std::vector<geometry_msgs::Pose> approach_poses, grasp_poses;
    std::vector<int> grasp_types(1, 1);   
    target_pose = g1::control::updatePose(object_pose, offset_vector(0), offset_vector(1), offset_vector(2));
    target_pose = g1::control::rotatePose(target_pose, 3.14, 1.57, 0);
    approach_poses.push_back(target_pose);

    offset_vector = 0.09 * x_dir; 
    target_pose = g1::control::updatePose(target_pose, offset_vector(0), offset_vector(1), offset_vector(2) );
    grasp_poses.push_back(target_pose);

    collision_objects.clear();
    collision_objects.push_back("microwave_handle");
    collision_objects.push_back("microwave_door");
    collision_objects.push_back("microwave_frame");
    pick_ptr->setTargetPicks(arm_side, object, approach_poses, grasp_poses, grasp_types);
    pick_ptr->setAllowedCollision(collision_objects);
    pick_ptr->setRetries(2);
    addActionTarget(pick_ptr);

    // Openning : Place action
    PlacePtr place_ptr(new Place("Opening microwave handle"));
    world_model_msgs::Object object_target = object;
    std::vector<geometry_msgs::Pose> place_poses, retreat_poses;
    offset_vector = -0.10 * x_dir + 0.01 * y_dir;
    target_pose = updatePose(target_pose, offset_vector(0), offset_vector(1), offset_vector(2));
    place_poses.push_back(target_pose);
    offset_vector = -0.10 * x_dir + 0.01 * y_dir ;
    target_pose = updatePose(target_pose, offset_vector(0), offset_vector(1), offset_vector(2));
    retreat_poses.push_back(target_pose);
    place_ptr->setAllowedCollision(collision_objects);
    place_ptr->setTargetPlaces(arm_side, object_target, place_poses, retreat_poses, grasp_types[0]);
    place_ptr->setRetries(2);
    addActionTarget(place_ptr);
    
    MovePtr move_ptr;

    // Lift action
    // MovePtr move_ptr(new Move("Lift hand"));
    // target_poses.clear();
    // offset_vector = 0.10 * y_dir + 0.10 * z_dir;
    // target_pose = updatePose(target_pose, offset_vector(0), offset_vector(1), offset_vector(2));
    // target_poses.push_back(target_pose);
    // move_ptr->setLockOrientation(arm_side, grasp_types[0]);
    // move_ptr->setTargetPoses(arm_side, target_poses);
    // addActionTarget(move_ptr);

    move_ptr.reset(new Move("Reset Arm"));
    move_ptr->setTargetJoints(arm_side, 
      std::vector<std::vector<double> > (1, init_arms_joints_[arm_side]));
    addActionTarget(move_ptr);

    move_ptr.reset(new Move("Above Handle"));
    target_poses.clear();
    offset_vector = -0.05 * x_dir + (0.05 + 0.45) * y_dir + (object_target.primitives[0].dimensions[2] / 2.0) * z_dir;
    target_pose = updatePose(object.primitive_poses[0], offset_vector(0), offset_vector(1), offset_vector(2));
    target_pose = rotatePose(target_pose, 3.14, 0.4, 0);
    target_poses.push_back(target_pose);
    move_ptr->setTargetPoses(arm_side, target_poses);
    move_ptr->setRetries(3);
    addActionTarget(move_ptr);
    
    move_ptr.reset(new Move("Moving Above Handle"));
    target_poses.clear();
    offset_vector = 0.02 * x_dir - 0.46 * y_dir;
    target_pose = updatePose(target_pose, offset_vector(0), offset_vector(1), offset_vector(2));
    target_poses.push_back(target_pose);
    move_ptr->setLockOrientation(arm_side);
    move_ptr->setTargetPoses(arm_side, target_poses);
    addActionTarget(move_ptr);

    move_ptr.reset(new Move("Inserting Handle"));
    target_poses.clear();
    offset_vector = -0.08 * z_dir;
    target_pose = updatePose(target_pose, offset_vector(0), offset_vector(1), offset_vector(2));
    target_poses.push_back(target_pose);
    move_ptr->setTargetPoses(arm_side, target_poses);
    move_ptr->setAllowedCollision(collision_objects);
    move_ptr->setLockOrientation(arm_side);
    addActionTarget(move_ptr);

    move_ptr.reset(new Move("Openning Door"));
    target_poses.clear();
    offset_vector = -0.10 * x_dir + 0.25 * y_dir;
    target_pose = updatePose(target_pose, offset_vector(0), offset_vector(1), offset_vector(2));
    target_poses.push_back(target_pose);
    move_ptr->setTargetPoses(arm_side, target_poses);
    move_ptr->setAllowedCollision(collision_objects);
    move_ptr->setLockOrientation(arm_side);
    move_ptr->setRetries(3);
    addActionTarget(move_ptr);

    move_ptr.reset(new Move("Retreating Hand"));
    target_poses.clear();
    offset_vector = 0.1 * z_dir;
    target_pose = updatePose(target_pose, offset_vector(0), offset_vector(1), offset_vector(2));
    target_poses.push_back(target_pose);
    move_ptr->setTargetPoses(arm_side, target_poses);
    move_ptr->setLockOrientation(arm_side);
    addActionTarget(move_ptr);

    move_ptr.reset(new Move("Moving Hand Away"));
    target_poses.clear();
    offset_vector = 0.10 * x_dir + 0.20 * y_dir;;
    target_pose = updatePose(target_pose, offset_vector(0), offset_vector(1), offset_vector(2));
    target_poses.push_back(target_pose);
    move_ptr->setTargetPoses(arm_side, target_poses);
    move_ptr->setLockOrientation(arm_side);
    addActionTarget(move_ptr);

    move_ptr.reset(new Move("Reset Arm"));
    move_ptr->setTargetJoints(arm_side, 
      std::vector<std::vector<double> > (1, init_arms_joints_[arm_side]));
    addActionTarget(move_ptr);

    return plan();
  }

  G1ControlErrorCode closeMicrowave(const world_model_msgs::Object &object)
  {

    ROS_INFO_STREAM("Use left hand to close microwave");
    std::string arm_side = "left";
    // resetArms(arm_side);

    geometry_msgs::Pose target_pose;
    std::vector<geometry_msgs::Pose> target_poses;
    std::vector<std::string> collision_objects;

    geometry_msgs::Pose object_pose = object.primitive_poses[0];
    Eigen::Quaternionf object_quat(object_pose.orientation.w,
      object_pose.orientation.x,
      object_pose.orientation.y,
      object_pose.orientation.z);
    Eigen::Matrix3f rotation = object_quat.matrix();
    Eigen::Vector3f x_dir = -1.0 * rotation.col(1);
    Eigen::Vector3f y_dir = rotation.col(0);
    Eigen::Vector3f z_dir = rotation.col(2);
    Eigen::Vector3f offset_vector;

    MovePtr move_ptr;
    move_ptr.reset(new Move("Move to the microwave door"));
    target_poses.clear();
    offset_vector = 0.20 * y_dir;
    target_pose = updatePose(object.primitive_poses[0], offset_vector(0), offset_vector(1), offset_vector(2));
    target_pose = rotatePose(target_pose, 3.14, 1.57, 1.57);
    target_poses.push_back(target_pose);
    move_ptr->setTargetPoses(arm_side, target_poses);
    move_ptr->setRetries(3);
    addActionTarget(move_ptr);
    
    move_ptr.reset(new Move("Closing the door"));
    target_poses.clear();
    offset_vector = - (object.primitives[0].dimensions[1] / 2.0 * 1.2 + 0.20) * y_dir;
    target_pose = updatePose(target_pose, offset_vector(0), offset_vector(1), offset_vector(2));
    target_poses.push_back(target_pose);
    move_ptr->setLockOrientation(arm_side);
    move_ptr->setTargetPoses(arm_side, target_poses);
    collision_objects.push_back(object.id);
    collision_objects.push_back("microwave_handle");
    collision_objects.push_back("microwave_frame");
    move_ptr->setAllowedCollision(collision_objects);
    addActionTarget(move_ptr);

    move_ptr.reset(new Move("Pushing the door"));
    target_poses.clear();
    offset_vector = (object.primitives[0].dimensions[1] / 2.0 + 0.03) * x_dir;
    target_pose = updatePose(target_pose, offset_vector(0), offset_vector(1), offset_vector(2));
    target_poses.push_back(target_pose);
    move_ptr->setLockOrientation(arm_side);
    move_ptr->setTargetPoses(arm_side, target_poses);
    move_ptr->setAllowedCollision(collision_objects);
    addActionTarget(move_ptr);

    collision_objects.clear();
    collision_objects.push_back(object.id);
    collision_objects.push_back("microwave_handle");
    move_ptr.reset(new Move("Retreating Hand"));
    target_poses.clear();
    offset_vector = - object.primitives[0].dimensions[1] / 2.0 * x_dir;
    target_pose = updatePose(target_pose, offset_vector(0), offset_vector(1), offset_vector(2));
    target_poses.push_back(target_pose);
    move_ptr->setLockOrientation(arm_side);
    move_ptr->setTargetPoses(arm_side, target_poses);
    move_ptr->setAllowedCollision(collision_objects);
    addActionTarget(move_ptr);

    move_ptr.reset(new Move("Reset Arm"));
    move_ptr->setTargetJoints(arm_side, 
      std::vector<std::vector<double> > (1, init_arms_joints_[arm_side]));
    move_ptr->setAllowedCollision(collision_objects);
    addActionTarget(move_ptr);

    return plan();
  }


  bool graspPlanning(const world_model_msgs::Object &object, 
    std::vector<geometry_msgs::Pose> &approach_poses,
    std::vector<geometry_msgs::Pose> &grasp_poses,
    std::vector<int> &grasp_types,
    const int &type_only,
    const int &type_filter)
  {
    approach_poses.clear();
    grasp_poses.clear();

    // Change ACM
    std::vector<std::string> collision_objects(1, object.id);
    // collision_objects.push_back("microwave_frame");
    // collision_objects.push_back("microwave_handle");
    planning_scene_interface_->addAllowedCollision(collision_objects);

    grasp_generator::GenerateGraspsGoal goal;
    world_model_msgs::Object transformed_object;
    if (!transformObjectPose(transformed_object, object, "base")) {
      return false;
    }
    goal.object = object;
    ROS_INFO_STREAM(object);

    reflex_planner_.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = reflex_planner_.waitForResult(ros::Duration(20.0));

    planning_scene_interface_->removeAllowedCollision(collision_objects);

    if (finished_before_timeout)
    {

      std::vector<grasp_generator::Grasp> grasps = reflex_planner_.getResult()->grasps;
      ROS_INFO_STREAM("Grasps found: " << grasps.size());
      for (int i = 0; i < grasps.size(); i++) {
        if (type_filter == grasps[i].grasp_type) {
          // ROS_INFO_STREAM("Filtering out the grasp type : " << type_filter);
          continue;
        }
        if (type_only != -1 && type_only != grasps[i].grasp_type) {
          // ROS_INFO_STREAM("Only use grasp type : " << type_only);
          continue;
        }

        approach_poses.push_back(grasps[i].pre_grasp_pose.pose);
        grasp_poses.push_back(grasps[i].grasp_pose.pose);
        grasp_types.push_back(grasps[i].grasp_type);
      }
      ROS_INFO_STREAM("Grasps found after high level filtering: " << grasp_poses.size());
      return grasp_poses.size() > 0;
    }
    else {
      ROS_INFO("Action did not finish before the time out.");
      return false;
    }

  }

  G1ControlErrorCode pickObject(const std::string &arm_use, const world_model_msgs::Object &object, int &grasp_type, bool enable_vertical)
  {
    // Grasp planning
    std::vector<geometry_msgs::Pose> approach_poses, grasp_poses;
    std::vector<int> grasp_types;
    int type_only = -1;
    int type_filter = -1;
    if (enable_vertical) {
      type_only = 3;
    } 
    else {
      type_filter = 3;
    }
 
    if (!graspPlanning(object, approach_poses, grasp_poses, grasp_types, type_only, type_filter)) {
      return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::FAILURE);
    }
  
    g1::control::PickPtr pick_ptr(new g1::control::Pick("Pick Object " + object.id));
    std::vector<std::string> collision_objects;
    collision_objects.push_back(object.id);
    pick_ptr->setAllowedCollision(collision_objects);
    pick_ptr->setTargetPicks(arm_use, object, approach_poses, grasp_poses, grasp_types);
    pick_ptr->setRetries(2);
    addActionTarget(pick_ptr);

    G1ControlErrorCode result = plan();
    
    if (result) {
      grasp_type = grasp_types[0];
    }

    return result;
  }

  // Place object on the table, assuming object is picked with grasp type and robot is stationary.
  G1ControlErrorCode placeObject(const std::string &arm_use,  
      const geometry_msgs::Point &location,
      world_model_msgs::Object &object,
      const int &grasp_type) 
  {
    geometry_msgs::Point preplace_offset = g1::control::constructPoint(0, 0, 0);

    // if (location.x != object.primitive_poses[0].position.x ||
    //     location.y != object.primitive_poses[0].position.y ||
    //     location.z != object.primitive_poses[0].position.z) {
    //   // Transfer object to target location
    //   geometry_msgs::Pose target_pose;
    //   std::vector<geometry_msgs::Pose> target_poses;
    //   g1::control::MovePtr move_ptr(new g1::control::Move("Lift arm"));
    //   target_pose = getCurrentPose(arm_use + "_gripper").pose;
    //   target_pose = g1::control::updatePose(target_pose, 0, 0, 0.2);
    //   target_poses.push_back(target_pose);
    //   move_ptr->setTargetPoses(arm_use, target_poses);
    //   move_ptr->setLockOrientation(arm_use, grasp_type);
    //   addActionTarget(move_ptr);

    // }
    preplace_offset = g1::control::constructPoint(0, 0, 0.2);

    // Place planning
    world_model_msgs::Object target_object = object;
    target_object.primitive_poses[0].position = location;

    //TODO: Orientation of the target object

    std::vector<geometry_msgs::Pose> place_poses, retreat_poses;
    std::vector<int> grasp_types;
    int type_only = grasp_type;
    int type_filter = -1;
    if (!graspPlanning(target_object, retreat_poses, place_poses, grasp_types, type_only, type_filter)) {
      return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::FAILURE);
    }

    g1::control::PlacePtr place_ptr(new g1::control::Place("Place Object " + object.id));
    std::vector<std::string> collision_objects;
    place_ptr->setTargetPlaces(arm_use, object, place_poses, retreat_poses, grasp_type, preplace_offset);
    place_ptr->setRetries(2);
    addActionTarget(place_ptr);

    return plan();
  }

  void updateObjectInfo(world_model_msgs::Object &object)
  {
    std::vector<std::string> object_ids(1, object.id);
    std::map<std::string, geometry_msgs::Pose> object_poses = planning_scene_interface_->getObjectPoses(object_ids);

    if (object_poses.count(object.id) > 0) {
      object.primitive_poses[0] = object_poses[object.id];
    }
    else {
      ROS_WARN_STREAM("Could not find object " << object.id << " in plannning scene");
    }
  }

  // Pick object from microwave, assuming the object is detected in the microwave and microwave is opened
  G1ControlErrorCode pickFromMicrowave(
      const std::string &arm_use,  
      const world_model_msgs::Object &object, 
      const world_model_msgs::Object &microwave_frame,
      int &grasp_type)
  {
    std::vector<std::string> collision_objects;
    std::vector<geometry_msgs::Pose> target_poses;
    geometry_msgs::Pose target_pose;
    g1::control::MovePtr move_ptr;

    // Grasp planning
    std::vector<geometry_msgs::Pose> approach_poses, grasp_poses;
    std::vector<int> grasp_types;
    int type_only = -1;
    int type_filter = 3;
 
    if (!graspPlanning(object, approach_poses, grasp_poses, grasp_types, type_only, type_filter)) {
      return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::FAILURE);
    }

    // Rotate the orientation of gripper
    if (grasp_types[0] == 1 || grasp_types[0] == 2) {
      target_pose = g1::control::rotatePose(microwave_frame.primitive_poses[1], 3.1415926, 3.1415926 / 2.0, 0);
    } 
    else if (grasp_types[0] == 0) {
      target_pose = g1::control::rotatePose(microwave_frame.primitive_poses[1], -1.57, 1.57, 0);
    }
    else if (grasp_types[0] == 4) {
      target_pose = g1::control::rotatePose(microwave_frame.primitive_poses[1], 1.57, -1.57, 0);
    }
    target_pose.position = getCurrentPose(arm_use + "_gripper").pose.position;

    move_ptr.reset(new g1::control::Move("Rotate gripper orientation"));
    target_poses.clear();
    target_poses.push_back(target_pose);
    move_ptr->setTargetPoses(arm_use, target_poses);
    move_ptr->setLockPosition(arm_use);
    move_ptr->setRetries(3);
    addActionTarget(move_ptr);

    // Compute location front of microwave
    geometry_msgs::Point location = microwave_frame.primitive_poses[1].position;

    Eigen::Quaternionf microwave_quat(microwave_frame.primitive_poses[1].orientation.w,
      microwave_frame.primitive_poses[1].orientation.x,
      microwave_frame.primitive_poses[1].orientation.y,
      microwave_frame.primitive_poses[1].orientation.z);
    Eigen::Matrix3f rotation = microwave_quat.matrix();
    Eigen::Vector3f x_dir = -1.0 * rotation.col(0);
    Eigen::Vector3f y_dir = -1.0 * rotation.col(1);
    Eigen::Vector3f z_dir = rotation.col(2);

    Eigen::Vector3f location_vector(location.x, location.y, location.z);
    location_vector += - (microwave_frame.primitives[1].dimensions[0] / 2.0 + 0.25) * x_dir 
                       + microwave_frame.primitives[2].dimensions[1] / 2.0 * y_dir
                       + (microwave_frame.primitives[1].dimensions[2] / 2.0 + 0.15) * z_dir;

    location.x = location_vector(0);
    location.y = location_vector(1);
    location.z = location_vector(2);

    target_pose.position = location;

    move_ptr.reset(new g1::control::Move("Place arm to front of microwave"));
    target_poses.clear();
    target_poses.push_back(target_pose);
    move_ptr->setTargetPoses(arm_use, target_poses);
    move_ptr->setLockOrientation(arm_use, grasp_types[0]);
    move_ptr->setRetries(3);
    addActionTarget(move_ptr);
  
    g1::control::PickPtr pick_ptr(new g1::control::Pick("Pick Object " + object.id));
    collision_objects.push_back(object.id);
    pick_ptr->setAllowedCollision(collision_objects);
    pick_ptr->setTargetPicks(arm_use, object, approach_poses, grasp_poses, grasp_types);
    pick_ptr->setRetries(2);
    addActionTarget(pick_ptr);

    // Lift the object
    geometry_msgs::Point offset_vector;
    move_ptr.reset(new g1::control::Move("Lift object"));
    offset_vector = constructPoint(0, 0, 0.02);
    move_ptr->setAllowedCollision(collision_objects);
    move_ptr->setTargetPoses(arm_use, offset_vector);
    move_ptr->setLockOrientation(arm_use, grasp_types[0]);
    move_ptr->setRetries(3);
    addActionTarget(move_ptr);

    // Retreat to front of microwave
    double height = 0;
    double length = 0;
    if (object.primitives[0].type == object.primitives[0].BOX) {
      height = object.primitives[0].dimensions[2];
      length = object.primitives[0].dimensions[0];
    }
    else if (object.primitives[0].type == object.primitives[0].CYLINDER) {
      height = object.primitives[0].dimensions[0];
      length = object.primitives[0].dimensions[1] * 2.0;
    }
    location_vector = - (length + 0.05) * x_dir;
    offset_vector = constructPoint(location_vector(0), location_vector(1), location_vector(2));

    move_ptr.reset(new g1::control::Move("Retreat arm to front of microwave"));
    move_ptr->setTargetPoses(arm_use, offset_vector);
    move_ptr->setLockOrientation(arm_use, grasp_types[0]);
    move_ptr->setAllowedCollision(collision_objects);
    move_ptr->setRetries(3);
    addActionTarget(move_ptr);

    // Retreat hand from microwave
    move_ptr.reset(new Move("Reset Arm"));
    move_ptr->setTargetJoints(arm_use, 
      std::vector<std::vector<double> > (1, init_arms_joints_grasp_[grasp_types[0]]));
    move_ptr->setLockOrientation(arm_use, grasp_types[0]);
    addActionTarget(move_ptr);

    G1ControlErrorCode result = plan();

    if (result) {
      grasp_type = grasp_types[0];
    }

    return result;
  }

  // Assuming object is picked, place into microwave
  G1ControlErrorCode placeIntoMicrowave(
      const std::string &arm_use,  
      const world_model_msgs::Object &object, 
      const world_model_msgs::Object &microwave_frame,
      const int &grasp_type)
  {
    std::vector<std::string> collision_objects;
    std::vector<geometry_msgs::Pose> target_poses;
    geometry_msgs::Pose target_pose;
    g1::control::MovePtr move_ptr;

    // Lift object TODO:: Correct orientation of object??
    move_ptr.reset(new g1::control::Move("Lift arm"));
    target_pose = getCurrentPose(arm_use + "_gripper").pose;
    target_pose = g1::control::updatePose(target_pose, 0, 0, 0.2);
    target_poses.push_back(target_pose);
    move_ptr->setTargetPoses(arm_use, target_poses);
    move_ptr->setLockOrientation(arm_use, grasp_type);
    addActionTarget(move_ptr);

    // Reset arm
    move_ptr.reset(new g1::control::Move("Reset arm"));
    move_ptr->setTargetJoints(arm_use, 
      std::vector<std::vector<double> > (1, init_arms_joints_grasp_[grasp_type]));
    move_ptr->setLockOrientation(arm_use, grasp_type);
    addActionTarget(move_ptr);

    // Compute location to put
    world_model_msgs::Object target_object = object;
    geometry_msgs::Point location = microwave_frame.primitive_poses[1].position;

    Eigen::Quaternionf microwave_quat(microwave_frame.primitive_poses[1].orientation.w,
      microwave_frame.primitive_poses[1].orientation.x,
      microwave_frame.primitive_poses[1].orientation.y,
      microwave_frame.primitive_poses[1].orientation.z);
    Eigen::Matrix3f rotation = microwave_quat.matrix();
    Eigen::Vector3f x_dir = -1.0 * rotation.col(0);
    Eigen::Vector3f y_dir = -1.0 * rotation.col(1);
    Eigen::Vector3f z_dir = rotation.col(2);

    Eigen::Vector3f location_vector(location.x, location.y, location.z);
    location_vector += - microwave_frame.primitives[1].dimensions[0] / 2.0 * x_dir 
                       + microwave_frame.primitives[2].dimensions[1] / 2.0 * y_dir
                       + microwave_frame.primitives[1].dimensions[2] / 2.0 * z_dir;

    // location.y += microwave_frame.primitives[2].dimensions[1] / 2.0;
    // location.z += microwave_frame.primitives[1].dimensions[2] / 2.0;
    // location.x -= microwave_frame.primitives[1].dimensions[0] / 2.0;

    double height = 0;
    double length = 0;
    if (target_object.primitives[0].type == target_object.primitives[0].BOX) {
      height = target_object.primitives[0].dimensions[2];
      length = target_object.primitives[0].dimensions[0];
    }
    else if (target_object.primitives[0].type == target_object.primitives[0].CYLINDER) {
      height = target_object.primitives[0].dimensions[0];
      length = target_object.primitives[0].dimensions[1] * 2.0;
    }

    location_vector += (length / 2.0 + 0.07) * x_dir + (height / 2.0) * z_dir;
    // location.z += height / 2.0;
    // location.x += length / 2.0 + 0.07;

    location.x = location_vector(0);
    location.y = location_vector(1);
    location.z = location_vector(2);

    target_object.primitive_poses[0].position = location;

    ROS_INFO_STREAM(target_object);

    // Place hand in front of microwave
    geometry_msgs::Point location_hand = location;

    location_vector += -(microwave_frame.primitives[1].dimensions[0] + 0.04) * x_dir + 0.1 * z_dir;
    // location_hand.x -= microwave_frame.primitives[1].dimensions[0] + 0.07;
    // location_hand.z += 0.1;
    location_hand.x = location_vector(0);
    location_hand.y = location_vector(1);
    location_hand.z = location_vector(2);

    if (grasp_type == 1 || grasp_type == 2) {
      target_pose = g1::control::rotatePose(microwave_frame.primitive_poses[1], 3.1415926, 3.1415926 / 2.0, 0);
    } 
    else if (grasp_type == 0) {
      target_pose = g1::control::rotatePose(microwave_frame.primitive_poses[1], -1.57, 1.57, 0);
    }
    else if (grasp_type == 0) {
      target_pose = g1::control::rotatePose(microwave_frame.primitive_poses[1], 1.57, -1.57, 0);
    }
    target_pose.position = location_hand;
    // target_pose.position = getCurrentPose(arm_use + "_gripper").pose.position;

    ROS_INFO_STREAM(target_pose);

    move_ptr.reset(new g1::control::Move("Place arm to front of microwave"));
    target_poses.clear();
    target_poses.push_back(target_pose);
    move_ptr->setTargetPoses(arm_use, target_poses);
    move_ptr->setLockOrientation(arm_use, grasp_type);
    move_ptr->setRetries(3);
    addActionTarget(move_ptr);
  
    // Place planning
    std::vector<geometry_msgs::Pose> place_poses, retreat_poses;
    std::vector<int> grasp_types;
    int type_only = grasp_type;
    int type_filter = -1;
    if (!graspPlanning(target_object, retreat_poses, place_poses, grasp_types, type_only, type_filter)) {
      return G1ControlErrorCode(g1_control_msgs::G1ControlErrorCodes::FAILURE);
    }

    geometry_msgs::Point preplace_offset = g1::control::constructPoint(0, 0, 0);
    g1::control::PlacePtr place_ptr(new g1::control::Place("Place Object " + object.id));
    place_ptr->setTargetPlaces(arm_use, object, place_poses, retreat_poses, grasp_type, preplace_offset);
    place_ptr->setRetries(2);
    addActionTarget(place_ptr);

    // Retreat to front of microwave
    move_ptr.reset(new g1::control::Move("Retreat arm to front of microwave"));
    move_ptr->setTargetPoses(arm_use, target_poses);
    move_ptr->setLockOrientation(arm_use, grasp_type);
    move_ptr->setRetries(3);
    addActionTarget(move_ptr);

    // Retreat hand from microwave
    move_ptr.reset(new Move("Reset Arm"));
    move_ptr->setTargetJoints(arm_use, 
      std::vector<std::vector<double> > (1, init_arms_joints_grasp_[grasp_type]));
    move_ptr->setLockOrientation(arm_use, grasp_type);
    move_ptr->setRetries(3);
    addActionTarget(move_ptr);

    // Retreat hand from microwave
    move_ptr.reset(new Move("Reset Arm"));
    move_ptr->setTargetJoints(arm_use, 
      std::vector<std::vector<double> > (1, init_arms_joints_[arm_use]));
    addActionTarget(move_ptr);

    // // Move to right of microwave
    // move_ptr.reset(new g1::control::Move("Move arm to the right of microwave"));
    // location_vector += -0.1 * y_dir;
    // target_pose.position.x = location_vector(0);
    // target_pose.position.y = location_vector(1);
    // target_pose.position.z = location_vector(2);
    // target_poses.clear();
    // target_poses.push_back(target_pose);
    // move_ptr->setTargetPoses(arm_use, target_poses);
    // move_ptr->setLockOrientation(arm_use, grasp_type);
    // move_ptr->setRetries(3);
    // addActionTarget(move_ptr);

    

    return plan();
  }

private:
  Options opt_;
  ros::NodeHandle node_handle_;
  PlanningSceneInterface *planning_scene_interface_;  
  moveit::planning_interface::MoveGroup *arms_controller_;
  HandController *left_hand_controller_, *right_hand_controller_;
  std::vector<ActionPtr> action_goals_;
  G1ControlState state_;
  actionlib::SimpleActionClient<grasp_generator::GenerateGraspsAction> reflex_planner_;

  std::map<std::string, std::vector<double> > init_arms_joints_;
  std::map<int, std::vector<double> > init_arms_joints_grasp_;

};

Executor::Executor(const Options &opt)
{
  impl_ = new ExecutorImpl(opt);
}
  
Executor::Executor(ros::NodeHandle &node_handle, const std::string &arm_side)
{
  impl_ = new ExecutorImpl(Options(node_handle, arm_side));
}

Executor::~Executor()
{
  delete impl_;
}

bool Executor::addActionTarget(ActionPtr action_goal)
{
  return impl_->addActionTarget(action_goal);
}

G1ControlErrorCode Executor::plan()
{
  return impl_->plan();
}

G1ControlErrorCode Executor::run()
{
  return impl_->run();
}

G1ControlErrorCode Executor::resetArms(const std::string &side)
{
  return impl_->resetArms(side);
}

G1ControlErrorCode Executor::resetGrippers(const std::string &side)
{
  return impl_->resetGrippers(side);
}

G1ControlErrorCode Executor::calibrateGrippers(const std::string &side)
{
  return impl_->calibrateGrippers(side);
}

G1ControlErrorCode Executor::initGrippers(const std::string &side)
{
  return impl_->initGrippers(side);
}

geometry_msgs::PoseStamped Executor::getCurrentPose(const std::string &end_effector_link)
{
  return impl_->getCurrentPose(end_effector_link);
}

void Executor::updatePlanningScene(const std::vector<world_model_msgs::Object> &objects)
{
  return impl_->updatePlanningScene(objects);
}

void Executor::updateObjectInfo(world_model_msgs::Object &object)
{
  return impl_->updateObjectInfo(object);
}

void Executor::clearPlanningScene()
{
  return impl_->clearPlanningScene();
}

G1ControlErrorCode Executor::openMicrowave(const world_model_msgs::Object &object)
{
  return impl_->openMicrowave(object);
}

G1ControlErrorCode Executor::closeMicrowave(const world_model_msgs::Object &object)
{
  return impl_->closeMicrowave(object);
}

void Executor::reset()
{
  return impl_->initialize();
}

G1ControlErrorCode Executor::pickObject(const std::string &arm_use, const world_model_msgs::Object &object, int &grasp_type, bool enable_vertical)
{
  return impl_->pickObject(arm_use, object, grasp_type, enable_vertical);
}

G1ControlErrorCode Executor::placeObject(const std::string &arm_use, 
  const geometry_msgs::Point &location,
  world_model_msgs::Object &object,
  const int &grasp_type)
{
  return impl_->placeObject(arm_use, location, object, grasp_type);
}

G1ControlErrorCode Executor::placeIntoMicrowave(
      const std::string &arm_use,  
      const world_model_msgs::Object &object, 
      const world_model_msgs::Object &microwave_frame,
      const int &grasp_type)
{
  return impl_->placeIntoMicrowave(arm_use, object, microwave_frame, grasp_type);
}

G1ControlErrorCode Executor::pickFromMicrowave(
      const std::string &arm_use,  
      const world_model_msgs::Object &object, 
      const world_model_msgs::Object &microwave_frame,
      int &grasp_type)
{
  return impl_->pickFromMicrowave(arm_use, object, microwave_frame, grasp_type);
}

}
}



