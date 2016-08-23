/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Tests the grasp generator filter
*/

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// MoveIt
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

// Rviz
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Grasp
#include <grasp_generator/simple_grasps.h>
#include <grasp_generator/grasp_filter.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Baxter specific properties
#include <grasp_generator/grasp_data.h>
// #include <grasp_generator/custom_environment2.h>

namespace grasp_generator
{

// Table data
static const double TABLE_HEIGHT = 1;
static const double TABLE_WIDTH = 1.6;
static const double TABLE_DEPTH = 0.8;
static const double TABLE_X = 0.72;
static const double TABLE_Y = 0;
static const double TABLE_Z = -0.69;

// static const double BLOCK_SIZE = 0.04;

class GraspGeneratorTest
{
private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Grasp generator
  grasp_generator::SimpleGraspsPtr simple_grasps_;

  // Tool for visualizing things in Rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // Grasp filter
  grasp_generator::GraspFilterPtr grasp_filter_;

  // data for generating grasps
  grasp_generator::GraspData grasp_data_;

  // Shared planning scene (load once for everything)
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // which baxter arm are we using
  std::string arm_;
  std::string ee_group_name_;
  std::string planning_group_name_;

public:

  // Constructor
  GraspGeneratorTest(int num_tests)
    : nh_("~")
  {
    // Get arm info from param server
    nh_.param("arm", arm_, std::string("left"));
    nh_.param("ee_group_name", ee_group_name_, std::string(arm_ + "_hand"));
    // ee_group_name_ = grasp_data_.ee_group_;
    planning_group_name_ = arm_ + "_arm";

    ROS_INFO_STREAM_NAMED("test","Arm: " << arm_);
    ROS_INFO_STREAM_NAMED("test","End Effector: " << ee_group_name_);
    ROS_INFO_STREAM_NAMED("test","Planning Group: " << planning_group_name_);

    // ---------------------------------------------------------------------------------------------
    // Load grasp data
    if (!grasp_data_.loadRobotGraspData(nh_, ee_group_name_))
      ros::shutdown();

    // ---------------------------------------------------------------------------------------------
    // Load planning scene to share
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    // ---------------------------------------------------------------------------------------------

    // Load the Robot Viz Tools for publishing to Rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(grasp_data_.base_link_, "/moveit_visual_tools", planning_scene_monitor_));
    visual_tools_->setLifetime(40.0);
    ros::Duration(1.0).sleep();
    // visual_tools_->loadEEMarker(grasp_data_.ee_group_);
    // visual_tools_->setFloorToBaseHeight(-0.9);

    // Clear out old collision objects just because
    visual_tools_->removeAllCollisionObjects();
    ros::Duration(1.0).sleep();

    // Create a collision table for fun
    // visual_tools_->publishCollisionTable(TABLE_X, TABLE_Y, 0, TABLE_WIDTH, TABLE_HEIGHT, TABLE_DEPTH, "table", rviz_visual_tools::GREEN);
    ros::Duration(1.0).sleep();

    // ---------------------------------------------------------------------------------------------
    // Load grasp generator
    simple_grasps_.reset( new grasp_generator::SimpleGrasps(visual_tools_) );

    // ---------------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------------
    // Load planning scene to share
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();
    ros::Duration(1).sleep();
    planning_scene_monitor_->updateSceneWithCurrentState();

    // Load grasp filter
    robot_state::RobotState robot_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
    grasp_filter_.reset(new grasp_generator::GraspFilter(robot_state, visual_tools_) );

    // ---------------------------------------------------------------------------------------------
    // Generate grasps for a bunch of random objects
    world_model_msgs::Object object;
    
    std::vector<grasp_generator::Grasp> possible_grasps;
    std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions, ik_solutions_pre; // save each grasps ik solution for visualization

    // Loop
    for (int i = 0; i < num_tests; ++i)
    {
      ROS_INFO_STREAM_NAMED("test","Adding random object " << i+1 << " of " << num_tests);

      // Remove randomness when we are only running one test
      if (num_tests == 1)
        generateTestObject(object);
      else
        generateRandomObject(object);

      geometry_msgs::Pose object_pose = object.primitive_poses[0];

      // Show the object
      // visual_tools_->publishBlock(object_pose, rviz_visual_tools::BLUE, BLOCK_SIZE);
      ROS_INFO_STREAM_NAMED("test", "Publishing object");
      // TODO(davetcoleman): use generateRandomCuboid()
      if (object.primitives[0].type == object.primitives[0].CYLINDER) {
        visual_tools_->publishCylinder(object.primitive_poses[0], rviz_visual_tools::BLUE, object.primitives[0].dimensions[0],2*object.primitives[0].dimensions[1]);
      }
      else if (object.primitives[0].type == object.primitives[0].BOX) {
        visual_tools_->publishCuboid(object.primitive_poses[0], object.primitives[0].dimensions[0], object.primitives[0].dimensions[1], object.primitives[0].dimensions[2], rviz_visual_tools::BLUE);
      }
      else{
        ROS_ERROR("Cannot publish the object!");
      }
      ros::Duration(1.0).sleep();

      possible_grasps.clear();
      ik_solutions.clear();
      ik_solutions_pre.clear();

      // Generate set of grasps for one object
      simple_grasps_->generateBlockGrasps( object, grasp_data_, possible_grasps);

      // Filter the grasp for only the ones that are reachable
      bool filter_pregrasps = true;
      grasp_filter_->filterGrasps(possible_grasps, ik_solutions, ik_solutions_pre, filter_pregrasps, grasp_data_.ee_parent_link_, planning_group_name_, planning_scene_monitor_);

      grasp_filter_->collisionFilterGrasps(possible_grasps, ik_solutions, ik_solutions_pre, planning_scene_monitor_, planning_group_name_);
      
      grasp_generator::Grasp best_grasp;
      grasp_filter_->chooseBestGrasp(possible_grasps, best_grasp);
      // ROS_INFO_STREAM_NAMED("test","+++++BestGrasp QUALITY: " << best_grasp.grasp_quality);
      // ROS_INFO_STREAM_NAMED("test","+++++BestGrasp type: " << (int) best_grasp.grasp_type);
      // ROS_INFO_STREAM_NAMED("test","+++++BestGrasp ID: " << best_grasp.grasp_id);
      //visualization
      for(int i = 0; i < possible_grasps.size(); ++i){
        //Debug visualization
        ROS_INFO_STREAM_NAMED("simple_grasp", "grasp poses display in rviz!");
        // visual_tools_->publishArrow(grasp_pose_msg.pose, rviz_visual_tools::RED);
        visual_tools_->publishArrow(possible_grasps[i].grasp_pose, rviz_visual_tools::RED);
        ros::Duration(0.01).sleep();
        ROS_INFO_STREAM_NAMED("simple_grasp", "pregrasp poses display in rviz!");
        // visual_tools_->publishArrow(pre_grasp_pose_msg.pose, rviz_visual_tools::BLUE);
        visual_tools_->publishArrow(possible_grasps[i].pre_grasp_pose, rviz_visual_tools::BLUE);
        ROS_INFO_STREAM_NAMED("intermediate value", "possible_grasps TYPE: " << (int) possible_grasps[i].grasp_type);
        ROS_INFO_STREAM_NAMED("intermediate value", "possible_grasps QUALITY: " << possible_grasps[i].grasp_quality);
        ROS_INFO_STREAM_NAMED("intermediate value", "possible_grasps ID: " << possible_grasps[i].grasp_id);
      }

      // Visualize them
      const robot_model::JointModelGroup* ee_jmg = planning_scene_monitor_->getRobotModel()->getJointModelGroup(grasp_data_.ee_group_);
        
      // visual_tools_->publishAnimatedGrasps(const std::vector<moveit_msgs::Grasp> grasps_result, const robot_model::JointModelGroup *ee_jmg, 0.01);
      const robot_model::JointModelGroup* arm_jmg = planning_scene_monitor_->getRobotModel()->getJointModelGroup(planning_group_name_);
      visual_tools_->publishIKSolutions(ik_solutions, arm_jmg, 0.4);

      // Make sure ros is still going
      if(!ros::ok())
        break;
    }


  }


  void generateTestObject(world_model_msgs::Object &object)
  {
    // rosparam get /world_model/initial_scene/
/*    primitive_poses:
     - [0.74, -0.47, -0.15, 0.0, 0.0, 0.0, 1.0]
     primitives:
     - dimensions: [0.07, 0.09]*/


     // rosservice call /world_model/get_states_objects "{object_id:"", manipulatable: true}"


    // // Position
    // geometry_msgs::Pose pose;
    // shape_msgs::SolidPrimitive shape;

    // shape.type = shape.CYLINDER;
    // shape.dimensions.resize(2);
    // shape.dimensions[shape.CYLINDER_HEIGHT]=0.05;
    // shape.dimensions[shape.CYLINDER_RADIUS]=0.06;


    // pose.position.x = 0.45;
    // pose.position.y = -0.5;
    // pose.position.z = -0.166;
    // // pose.position.z = -0.205260;



    // // pose.position.x = 0.84;
    // // pose.position.y = -0.27;
    // // pose.position.z = -0.15;

    // pose.orientation.x = 0;
    // pose.orientation.y = 0;
    // pose.orientation.z = 0;
    // pose.orientation.w = 1;

    // // Choose which object to test
    // object.primitive_poses.push_back(pose);
    // object.primitives.push_back(shape);
    // object.id = "bowl";


    // // Position
    // geometry_msgs::Pose pose;
    // shape_msgs::SolidPrimitive shape;

    // shape.type = shape.CYLINDER;
    // shape.dimensions.resize(2);
    // //type 1
    // shape.dimensions[shape.CYLINDER_HEIGHT]=0.08;
    // shape.dimensions[shape.CYLINDER_RADIUS]=0.08;

    // //type 2
    // // shape.dimensions[shape.CYLINDER_HEIGHT]=0.08;
    // // shape.dimensions[shape.CYLINDER_RADIUS]=0.04;


    // pose.position.x = visual_tools_->dRand(TABLE_X-TABLE_DEPTH/2.0 , TABLE_X);
    // pose.position.y = visual_tools_->dRand(TABLE_Y - TABLE_WIDTH/2.0   , TABLE_Y);
    // pose.position.z = TABLE_Z + TABLE_HEIGHT / 2.0 +  shape.dimensions[shape.CYLINDER_HEIGHT] /2.0;

    // double angle = M_PI * visual_tools_->dRand(0.1,1);
    // Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    // pose.orientation.x = quat.x();
    // pose.orientation.y = quat.y();
    // pose.orientation.z = quat.z();
    // pose.orientation.w = quat.w();

    // // Choose which object to test
    // object.primitive_poses.push_back(pose);
    // object.primitives.push_back(shape);
    // object.id = "bowl";

    // Position
    geometry_msgs::Pose pose;
    shape_msgs::SolidPrimitive shape;

    shape.type = shape.BOX;
    shape.dimensions.resize(3);
    shape.dimensions[shape.BOX_X]=0.10;
    shape.dimensions[shape.BOX_Y]=0.20;
    shape.dimensions[shape.BOX_Z]=0.3;


    pose.position.x = 0.64;
    pose.position.y = -0.47;
    pose.position.z = 0.15;

    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    // Choose which object to test
    object.primitive_poses.push_back(pose);
    object.primitives.push_back(shape);
    object.id = "bowl";
  }


  void generateRandomObject(world_model_msgs::Object &object)
  {
    // Position
    geometry_msgs::Pose pose;
    shape_msgs::SolidPrimitive shape;

    shape.type = shape.CYLINDER;
    shape.dimensions.resize(2);
    //type 1
    shape.dimensions[shape.CYLINDER_HEIGHT]=0.08;
    shape.dimensions[shape.CYLINDER_RADIUS]=0.08;

    //type 2
    shape.dimensions[shape.CYLINDER_HEIGHT]=0.08;
    shape.dimensions[shape.CYLINDER_RADIUS]=0.04;


    pose.position.x = visual_tools_->dRand(TABLE_X-TABLE_DEPTH/2.0  , TABLE_X);
    pose.position.y = visual_tools_->dRand(TABLE_Y  , TABLE_Y + TABLE_WIDTH/2.0 );
    pose.position.z = TABLE_Z + TABLE_HEIGHT / 2.0 +  shape.dimensions[shape.CYLINDER_HEIGHT] /2.0;

    double angle = M_PI * visual_tools_->dRand(0.1,1);
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();

    // Choose which object to test
    object.primitive_poses.push_back(pose);
    object.primitives.push_back(shape);
    object.id = "bowl";
  }

}; // end of class

} // namespace


int main(int argc, char *argv[])
{
  int num_tests = 1;

  ros::init(argc, argv, "grasp_generator_test");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Check for verbose flag
  bool verbose = false;
  if (argc > 1)
  {
    for (std::size_t i = 0; i < argc; ++i)
    {
      if (strcmp(argv[i], "--verbose") == 0)
      {
        ROS_INFO_STREAM_NAMED("main","Running in VERBOSE mode (much slower)");
        verbose = true;
      }
    }
  }

  // Seed random
  srand(ros::Time::now().toSec());

  // Benchmark time
  ros::Time start_time;
  start_time = ros::Time::now();

  // Run Tests
  grasp_generator::GraspGeneratorTest tester(num_tests);

  // Benchmark time
  double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
  ROS_INFO_STREAM_NAMED("","Total time: " << duration);
  std::cout << duration << "\t" << num_tests << std::endl;

  ros::Duration(1.0).sleep(); // let rviz markers finish publishing

  return 0;
}
