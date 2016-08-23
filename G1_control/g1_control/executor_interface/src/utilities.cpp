/*
 * =====================================================================================
 *
 *       Filename:  utilities.cpp
 *
 *    Description:  utilities functions
 *
 *        Version:  1.0
 *        Created:  06/16/2016 11:02:36 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ren Mao (neroam), neroam@umd.edu
 *   Organization:  
 *
 * =====================================================================================
 */

#include <g1_control/utilities.h>
#include <tf/transform_datatypes.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>

namespace g1
{

namespace control
{
  geometry_msgs::Point constructPoint(const float &x, const float &y, const float &z)
  {
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;

    return point;
  }

  geometry_msgs::Quaternion constructQuat(const float &x, const float &y, const float &z, const float &w)
  {
    geometry_msgs::Quaternion quat;
    quat.x = x;
    quat.y = y;
    quat.z = z;
    quat.w = w;

    return quat;
  }

  geometry_msgs::Pose constructPose(const geometry_msgs::Point &position, const geometry_msgs::Quaternion &orientation)
  {
    geometry_msgs::Pose pose;
    pose.position = position;
    pose.orientation = orientation;

    return pose;
  }

  std::map<std::string, double> listToJoints(const std::string &side, const std::vector<double> &list) {
    std::string joint_names[7] = {"s0", "s1", "e0", "e1", "w0", "w1", "w2"};
    std::map<std::string, double> joints_map;
    for (int i = 0; i < 7; i++) {
      joints_map[ side + "_" + joint_names[i] ] = list[i];
    }

    return joints_map;
  }

  // Utility functions
  moveit::planning_interface::MoveItErrorCode planRetry(
      moveit::planning_interface::MoveGroup &group,
      moveit::planning_interface::MoveGroup::Plan &plan,
      const int &retries)
  {
    moveit::planning_interface::MoveItErrorCode result(moveit_msgs::MoveItErrorCodes::FAILURE);
    for (int i = 0; i < retries; i++) {
      ROS_INFO("Trial %d", i+1);
      result = group.plan(plan);
      if (result) {
        if (isPlanValid(plan)) {
          return result;
        }
        else {
          ROS_INFO("Plan rejected due to discountinuity!");
        }
      }
    }
    return moveit::planning_interface::MoveItErrorCode(moveit_msgs::MoveItErrorCodes::FAILURE);
  }

  bool isPlanValid(moveit::planning_interface::MoveGroup::Plan &plan)
  {
    std::vector<trajectory_msgs::JointTrajectoryPoint> points = plan.trajectory_.joint_trajectory.points;

    double score_max = 0;
    for (int i = 0; i < points.size() - 1; i++) {
      double score = 0; 
      for (int j = 0; j < points[i].positions.size(); j++) {
        score += pow((points[i].positions[j] - points[i+1].positions[j]), 2.0);
      }
      if (score > score_max) {
        score_max = score;
      }
    }

    ROS_INFO_STREAM("Plan scored with dicontinuity: " << score_max);
    if (score_max > 1) {
      return false;
    }
    else return true;
  }

  moveit_msgs::RobotState getEndState(const moveit::planning_interface::MoveGroup::Plan &plan) 
  {
    moveit_msgs::RobotState end_state;

    end_state.joint_state.header = plan.trajectory_.joint_trajectory.header;
    end_state.joint_state.name = plan.trajectory_.joint_trajectory.joint_names;
    end_state.joint_state.position = plan.trajectory_.joint_trajectory.points.back().positions;
    end_state.joint_state.velocity = plan.trajectory_.joint_trajectory.points.back().velocities;
    end_state.joint_state.effort = plan.trajectory_.joint_trajectory.points.back().effort;
    end_state.multi_dof_joint_state.header = plan.trajectory_.multi_dof_joint_trajectory.header;
    end_state.multi_dof_joint_state.joint_names = plan.trajectory_.multi_dof_joint_trajectory.joint_names;

    return end_state;
  }

  moveit_msgs::CollisionObject toCollisionObject(
      moveit::planning_interface::MoveGroup &group,
      const world_model_msgs::Object &object)
  {
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = group.getPlanningFrame();

    collision_object.id = object.id;
    collision_object.primitives = object.primitives;
    collision_object.primitive_poses = object.primitive_poses;
    collision_object.operation = collision_object.ADD;
    return collision_object;
  }

//   void mapToCollisionObjects(
//       moveit::planning_interface::MoveGroup &group,
//       const std::map<std::string, geometry_msgs::Pose> &object_poses,
//       std::vector<moveit_msgs::CollisionObject> &objects)
//   {
// 
//     objects.clear();
//     moveit_msgs::CollisionObject collision_object;
//     collision_object.header.frame_id = group.getPlanningFrame();
//     collision_object.operation = collision_object.ADD;
// 
//     std::map<std::string, geometry_msgs::Pose>::iterator it;
//     for (it = object_poses.begin(); it != object_poses.end(); ++it) {
//     collision_object.id = object.id;
//     collision_object.primitives = object.primitives;
//     collision_object.primitive_poses = object.primitive_poses;
//     
// 
//       objects.push_back()
//       
//     }
// 
// 
//     collision_object.id = object.id;
//     collision_object.primitives = object.primitives;
//     collision_object.primitive_poses = object.primitive_poses;
//     collision_object.operation = collision_object.ADD;
//     return collision_object;
//   }

  geometry_msgs::Pose translatePose(
      const geometry_msgs::Pose &pose,
      const float &x,
      const float &y,
      const float &z
      )
  {
    geometry_msgs::Pose result = pose;
    result.position.x = x;
    result.position.y = y;
    result.position.z = z;
    return result;
  }

  geometry_msgs::Pose updatePose(
      const geometry_msgs::Pose &pose,
      const float &dx,
      const float &dy,
      const float &dz
      )
  {
    geometry_msgs::Pose result = pose;
    result.position.x += dx;
    result.position.y += dy;
    result.position.z += dz;
    return result;
  }

  geometry_msgs::Pose rotatePose(
      const geometry_msgs::Pose &pose,
      const float &r,
      const float &p,
      const float &y
      )
  {
    geometry_msgs::Pose result = pose;
    tf::Quaternion rotation = tf::createQuaternionFromRPY(r,p,y);
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(result.orientation, orientation);
    tf::quaternionTFToMsg(orientation*rotation, result.orientation);
    return result;
  }
  
  bool getPosefromTF(
        geometry_msgs::Pose &transformed_pose,
        tf::TransformListener &tf_listener,
        const std::string &destination_frame,
        const std::string &original_frame,
        const double &duration
      )
  {
    tf::StampedTransform transform;
    try {
      tf_listener.waitForTransform(destination_frame, original_frame, ros::Time(0), ros::Duration(duration) );
      tf_listener.lookupTransform(destination_frame, original_frame, ros::Time(0), transform);
    } catch (tf::TransformException ex) {
      ROS_WARN("%s",ex.what());
      return false;
    }

    transformed_pose.position.x = transform.getOrigin().x();
    transformed_pose.position.y = transform.getOrigin().y();
    transformed_pose.position.z = transform.getOrigin().z();
    tf::quaternionTFToMsg(transform.getRotation(), transformed_pose.orientation);
    return true;
  }

  bool transformObjectPose(
        world_model_msgs::Object &transformed_object,
        const world_model_msgs::Object &src_object,
        const std::string &destination_frame,
        const double &duration
      )
  {
    tf::TransformListener listener;
    tf::StampedTransform transform;
    std::string original_frame = "map";
    try {
      listener.waitForTransform(destination_frame, original_frame, ros::Time(0), ros::Duration(duration) );
      listener.lookupTransform(destination_frame, original_frame, ros::Time(0), transform);
    } catch (tf::TransformException ex) {
      ROS_WARN("%s",ex.what());
      return false;
    }

    transformed_object = src_object;

    for (int i = 0; i < src_object.primitive_poses.size(); i++) {
      tf::Transform object_transform, result_transform;
      object_transform.setOrigin(tf::Vector3(
          src_object.primitive_poses[i].position.x,
          src_object.primitive_poses[i].position.y,
          src_object.primitive_poses[i].position.z));
      tf::Quaternion orientation;
      tf::quaternionMsgToTF(src_object.primitive_poses[i].orientation, orientation);
      object_transform.setRotation(orientation);

      result_transform = transform*object_transform;
      transformed_object.primitive_poses[i].position.x = result_transform.getOrigin().x();
      transformed_object.primitive_poses[i].position.y = result_transform.getOrigin().y();
      transformed_object.primitive_poses[i].position.z = result_transform.getOrigin().z();

      tf::quaternionTFToMsg(result_transform.getRotation(), transformed_object.primitive_poses[i].orientation);
    }
    return true;
  }

  bool publishPosetoTF(
        geometry_msgs::Pose &transformed_pose,
        tf::TransformBroadcaster &tf_broadcaster,
        const std::string &destination_frame,
        const std::string &original_frame
      )
  {
    tf::Transform transform;
    transform.setOrigin(
      tf::Vector3(transformed_pose.position.x,
                  transformed_pose.position.y,
                  transformed_pose.position.z));
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(transformed_pose.orientation, orientation);
    transform.setRotation(orientation);
    tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), destination_frame, original_frame));

    return true;
  }

  geometry_msgs::Pose getPosefromRSMsg(
      moveit::planning_interface::MoveGroup &group,
      const moveit_msgs::RobotState &robot_state,
      const std::string &end_effector
      )
  {
    Eigen::Affine3d pose;
    pose.setIdentity();
    if (end_effector.empty()) {
      ROS_ERROR("No end-effector specified.");
    }
    else {
      moveit::core::RobotState state(*group.getCurrentState());
      moveit::core::robotStateMsgToRobotState(robot_state, state);
      const moveit::core::LinkModel *lm = state.getLinkModel(end_effector);

      if (lm) {
        pose = state.getGlobalLinkTransform(lm);
      }
    }

    geometry_msgs::Pose result;
    tf::poseEigenToMsg(pose, result);
    return result;
  }

  void getTFfromPose(
      const geometry_msgs::Pose &pose,
      tf::Transform &transform
      )
  {
    transform.setOrigin(
      tf::Vector3(pose.position.x,
                  pose.position.y,
                  pose.position.z));
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(pose.orientation, orientation);
    transform.setRotation(orientation);
  }

  geometry_msgs::Pose getRelativePose(
      const geometry_msgs::Pose &dest,
      const geometry_msgs::Pose &src
      )
  {
    tf::Transform tf_dest, tf_src, transform;
    getTFfromPose(dest, tf_dest);
    getTFfromPose(src, tf_src);
    transform = tf_src.inverseTimes(tf_dest);

    geometry_msgs::Pose transformed_pose;
    transformed_pose.position.x = transform.getOrigin().x();
    transformed_pose.position.y = transform.getOrigin().y();
    transformed_pose.position.z = transform.getOrigin().z();
    tf::quaternionTFToMsg(transform.getRotation(), transformed_pose.orientation);
    return transformed_pose; 
  }


  
}

}

