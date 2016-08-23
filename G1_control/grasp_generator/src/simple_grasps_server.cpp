/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013-2014, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Bence Magyar
   Desc:   Action server wrapper for object grasp generator. Currently only works for REEM robot,
           needs to be changed to work with yaml configuration file instead.
*/

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <actionlib/server/simple_action_server.h>

// Grasp generation
#include <grasp_generator/simple_grasps.h>
#include <grasp_generator/GenerateGraspsAction.h>

#include <moveit_visual_tools/moveit_visual_tools.h>



// Baxter specific properties
#include <grasp_generator/grasp_data.h>
#include <grasp_generator/grasp_filter.h>
// #include <grasp_generator/custom_environment2.h>



namespace grasp_generator
{
  class GraspGeneratorServer
  {
  protected:
    // A shared node handle
    ros::NodeHandle nh_;

    // Action server
    actionlib::SimpleActionServer<grasp_generator::GenerateGraspsAction> as_;

    std::string action_name_;
    // create messages that are used to published feedback/result
    grasp_generator::GenerateGraspsFeedback feedback_;
    grasp_generator::GenerateGraspsResult result_;

  public:
    // Grasp generator
    grasp_generator::SimpleGraspsPtr simple_grasps_;

    // Shared planning scene
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

    // class for publishing stuff to rviz
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

    // Grasp filter
    grasp_generator::GraspFilterPtr grasp_filter_;

    // robot-specific data for generating grasps
    grasp_generator::GraspData grasp_data_;

    // which arm are we using
    std::string side_;
    std::string planning_group_name_;

    // Constructor
    GraspGeneratorServer(const std::string &name, const std::string &side) :
      nh_("~"),
      as_(nh_, name, boost::bind(&grasp_generator::GraspGeneratorServer::executeCB, this, _1), false),
      action_name_(name),
      side_(side),
      planning_group_name_(side_+"_arm"){

      if (name == "parallel"){
        //
      }
      else{

        ros::Rate r(1);

        // ---------------------------------------------------------------------------------------------
        // Load grasp data specific to our robot
        if (!grasp_data_.loadRobotGraspData(nh_, side_+"_hand"))
          ros::shutdown();

        // ---------------------------------------------------------------------------------------------
        // Load the Robot Viz Tools for publishing to Rviz
        visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(grasp_data_.base_link_,"/moveit_visual_tools"));
        // r.sleep();

        visual_tools_->setLifetime(100.0);
        // visual_tools_->loadEEMarker(grasp_data_.ee_group_);

        simple_grasps_.reset( new grasp_generator::SimpleGrasps(visual_tools_,false) );
        // r.sleep();

      // ---------------------------------------------------------------------------------------------
      // Load planning scene to share
      planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
      planning_scene_monitor_->startSceneMonitor();
      planning_scene_monitor_->startStateMonitor();
      planning_scene_monitor_->startWorldGeometryMonitor();
      ros::Duration(1).sleep();
      planning_scene_monitor_->updateSceneWithCurrentState();

        // ---------------------------------------------------------------------------------------------
        // Load grasp filter
        robot_state::RobotState robot_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();

        grasp_filter_.reset(new grasp_generator::GraspFilter(robot_state, visual_tools_) );

        as_.start();
      }
    }

    ~GraspGeneratorServer(void) {}


    void executeCB(const grasp_generator::GenerateGraspsGoalConstPtr &goal)
    {
      // ros::Rate r(1);

      std::vector<grasp_generator::Grasp> possible_grasps;
      possible_grasps.clear();
      // ---------------------------------------------------------------------------------------------
      // Remove previous results
      result_.grasps.clear();

      std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions, ik_solutions_pre;
      ik_solutions.clear();
      ik_solutions_pre.clear();
      
      // // ---------------------------------------------------------------------------------------------
      // // Set object width and generate grasps
      // grasp_data_.object_size_ = goal->width;

      // // Generate grasps for all options that were passed
      // grasp_axis_t axis;
      // grasp_direction_t direction;
      // grasp_rotation_t rotation;
      // for(size_t i=0; i<goal->options.size(); ++i)
      // {
        // graspGeneratorOptions2Inner(goal->options[i], axis, direction, rotation);

      bool filter_pregrasps = true;

      // Show the block
      // ROS_INFO_STREAM_NAMED("test", "-------------------Publishing object");
      // r.sleep();
      // visual_tools_->publishCylinder(goal->object.primitive_poses[0], rviz_visual_tools::BLUE, goal->object.primitives[0].dimensions[0], (goal->object.primitives[0].dimensions[1])*2.0, "target_bowl");
      // ROS_INFO_STREAM_NAMED(" zxdf", "-------goal.object: ---------" << goal->object.primitives[0].dimensions[0] << goal->object.primitives[0].dimensions[1]);

      simple_grasps_->generateAxisGrasps(goal->object, grasp_data_, possible_grasps, planning_scene_monitor_);

      grasp_filter_->filterGrasps(possible_grasps, ik_solutions, ik_solutions_pre, filter_pregrasps, grasp_data_.ee_parent_link_, planning_group_name_, planning_scene_monitor_);

      grasp_filter_->collisionFilterGrasps(possible_grasps, ik_solutions, ik_solutions_pre, planning_scene_monitor_, planning_group_name_);

      grasp_generator::Grasp best_grasp;
      grasp_filter_->chooseBestGrasp(possible_grasps, best_grasp);
      result_.grasps = possible_grasps;

      // //visualization
      // for(int i = 0; i < possible_grasps.size(); ++i){
      //   //Debug visualization
      //   ROS_INFO_STREAM_NAMED("simple_grasp", "grasp poses display in rviz!");
      //   // visual_tools_->publishArrow(grasp_pose_msg.pose, rviz_visual_tools::RED);
      //   visual_tools_->publishArrow(possible_grasps[i].grasp_pose, rviz_visual_tools::RED);
      //   ros::Duration(0.01).sleep();
      //   ROS_INFO_STREAM_NAMED("simple_grasp", "pregrasp poses display in rviz!");
      //   // visual_tools_->publishArrow(pre_grasp_pose_msg.pose, rviz_visual_tools::BLUE);
      //   visual_tools_->publishArrow(possible_grasps[i].pre_grasp_pose, rviz_visual_tools::BLUE);
      //   ROS_INFO_STREAM_NAMED("intermediate value", "possible_grasps TYPE: " << (int) possible_grasps[i].grasp_type);
      //   ROS_INFO_STREAM_NAMED("intermediate value", "possible_grasps QUALITY: " << possible_grasps[i].grasp_quality);
      //   ROS_INFO_STREAM_NAMED("intermediate value", "possible_grasps ID: " << possible_grasps[i].grasp_id);
      // }

      // // Visualize them
      // const robot_model::JointModelGroup* ee_jmg = planning_scene_monitor_->getRobotModel()->getJointModelGroup(grasp_data_.ee_group_);
        
      // // visual_tools_->publishAnimatedGrasps(const std::vector<moveit_msgs::Grasp> grasps_result, const robot_model::JointModelGroup *ee_jmg, 0.01);
      // const robot_model::JointModelGroup* arm_jmg = planning_scene_monitor_->getRobotModel()->getJointModelGroup(planning_group_name_);
      // visual_tools_->publishIKSolutions(ik_solutions, arm_jmg, 0.4);

      // // Visualize them
      // const robot_model::JointModelGroup* ee_jmg = planning_scene_monitor_->getRobotModel()->getJointModelGroup(grasp_data_.ee_group_);
      // // visual_tools_->publishAnimatedGrasps(possible_grasps, ee_jmg);
      // const robot_model::JointModelGroup* arm_jmg = planning_scene_monitor_->getRobotModel()->getJointModelGroup(planning_group_name_);
      // visual_tools_->publishIKSolutions(ik_solutions, arm_jmg, 0.4);
      // }
      // fallback behaviour, generate default grasps when no options were passed
      // if(goal->options.empty())
      // {
      //   simple_grasps_->generateBlockGrasps(goal->pose, grasp_data_, result_.grasps);
      // }

      // ---------------------------------------------------------------------------------------------
      // Publish results
      as_.setSucceeded(result_);
    }

  };
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "grasp_generator_server");
  grasp_generator::GraspGeneratorServer grasp_generator_server("reflex", "right");
  // grasp_generator::GraspGeneratorServer grasp_generator_server("parallel", "right");

  ros::spin();
  return 0;
}
