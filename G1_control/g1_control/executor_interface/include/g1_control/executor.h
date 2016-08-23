#ifndef G1_CONTROL_EXECUTOR_
#define G1_CONTROL_EXECUTOR_

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <g1_control_msgs/G1ControlErrorCodes.h>
#include <g1_control_msgs/G1ControlStateCodes.h>
#include <world_model_msgs/Object.h>
#include <g1_control/action.h>

namespace g1
{

namespace control
{

class Executor
{
public:

  struct Options
  {
    Options(ros::NodeHandle &node_handle,
            const std::string &arm_side,
            const std::string &left_hand_type = std::string(""),
            const std::string &right_hand_type = std::string(""))
      : arm_side_(arm_side),
        left_hand_type_(left_hand_type),
        right_hand_type_(right_hand_type),
        node_handle_(node_handle)
    {
    }

    std::string arm_side_;
    std::string left_hand_type_;
    std::string right_hand_type_;
    ros::NodeHandle node_handle_;
  };

  Executor(ros::NodeHandle &node_handle, const std::string &arm_side);

  Executor(const Options &opt);

  ~Executor();

  bool addActionTarget(ActionPtr action_goal);
  
  G1ControlErrorCode plan();

  G1ControlErrorCode run();

  void reset();

  G1ControlErrorCode resetArms(const std::string &side = "both");
  G1ControlErrorCode resetGrippers(const std::string &side = "both");
  G1ControlErrorCode calibrateGrippers(const std::string &side = "both");
  G1ControlErrorCode initGrippers(const std::string &side = "both");

  geometry_msgs::PoseStamped getCurrentPose(const std::string &end_effector_link = "");

  void updatePlanningScene(const std::vector<world_model_msgs::Object> &objects);
  void clearPlanningScene();
  void updateObjectInfo(world_model_msgs::Object &object);


  G1ControlErrorCode openMicrowave(const world_model_msgs::Object &object);
  G1ControlErrorCode pickObject(const std::string &arm_use, const world_model_msgs::Object &object, int &grasp_type, bool enable_vertical = false);
  G1ControlErrorCode placeObject(const std::string &arm_use,
      const geometry_msgs::Point &location,
      world_model_msgs::Object &object,
      const int &grasp_type);

  G1ControlErrorCode placeIntoMicrowave(
      const std::string &arm_use,  
      const world_model_msgs::Object &object, 
      const world_model_msgs::Object &microwave_frame,
      const int &grasp_type);

  G1ControlErrorCode closeMicrowave(const world_model_msgs::Object &object);

  G1ControlErrorCode pickFromMicrowave(
      const std::string &arm_use,  
      const world_model_msgs::Object &object, 
      const world_model_msgs::Object &microwave_frame,
      int &grasp_type);

private:
  class ExecutorImpl;
  ExecutorImpl *impl_;

};
  
}
}

#endif


