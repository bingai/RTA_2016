#ifndef G1_CONTROL_UTILITIES_
#define G1_CONTROL_UTILITIES_
  
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/RobotState.h>
#include <world_model_msgs/Object.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


namespace g1
{

namespace control
{
  geometry_msgs::Point constructPoint(const float &x, const float &y, const float &z);
  geometry_msgs::Quaternion constructQuat(const float &x, const float &y, const float &z, const float &w);
  geometry_msgs::Pose constructPose(
      const geometry_msgs::Point &position, 
      const geometry_msgs::Quaternion &orientation); 


  std::map<std::string, double> listToJoints(
      const std::string &side, 
      const std::vector<double> &list);

  moveit::planning_interface::MoveItErrorCode planRetry(
      moveit::planning_interface::MoveGroup &group,
      moveit::planning_interface::MoveGroup::Plan &plan,
      const int &retries = 1);

  bool isPlanValid(moveit::planning_interface::MoveGroup::Plan &plan);
  moveit_msgs::RobotState getEndState(const moveit::planning_interface::MoveGroup::Plan &plan);

  moveit_msgs::CollisionObject toCollisionObject(
      moveit::planning_interface::MoveGroup &group,
      const world_model_msgs::Object &object);

  geometry_msgs::Pose translatePose(
      const geometry_msgs::Pose &pose,
      const float &x,
      const float &y,
      const float &z
      );

  geometry_msgs::Pose updatePose(
      const geometry_msgs::Pose &pose,
      const float &dx,
      const float &dy,
      const float &dz
      );

  geometry_msgs::Pose rotatePose(
      const geometry_msgs::Pose &pose,
      const float &r,
      const float &p,
      const float &y
      );

  bool getPosefromTF(
        geometry_msgs::Pose &transformed_pose,
        tf::TransformListener &tf_listener,
        const std::string &destination_frame,
        const std::string &original_frame,
        const double &duration = 2.0
      );

  bool publishPosetoTF(
        geometry_msgs::Pose &transformed_pose,
        tf::TransformBroadcaster &tf_broadcaster,
        const std::string &destination_frame,
        const std::string &original_frame
      );

  geometry_msgs::Pose getPosefromRSMsg(
      moveit::planning_interface::MoveGroup &group,
      const moveit_msgs::RobotState &robot_state,
      const std::string &end_effector
      );

  geometry_msgs::Pose getRelativePose(
      const geometry_msgs::Pose &dest,
      const geometry_msgs::Pose &src
      );

  void getTFfromPose(
      const geometry_msgs::Pose &pose,
      tf::Transform &transform
      );

  bool transformObjectPose(
        world_model_msgs::Object &transformed_object,
        const world_model_msgs::Object &src_object,
        const std::string &destination_frame,
        const double &duration = 2.0
      );

}

}



#endif
