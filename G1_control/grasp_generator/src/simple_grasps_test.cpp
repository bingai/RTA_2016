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
   Desc:   Tests the grasp generator
*/

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>


#include <world_model_msgs/Object.h>

// Grasp generation
#include <grasp_generator/simple_grasps.h>

// Baxter specific properties
// #include <grasp_generator/custom_environment2.h>

// #include <world_model_msgs/Object.h>
// #include <shape_msgs/SolidPrimitive.h>


namespace baxter_pick_place
{

// static const double BLOCK_SIZE = 0.04;

class GraspGeneratorTest
{
private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Grasp generator
  grasp_generator::SimpleGraspsPtr simple_grasps_;

  // class for publishing stuff to rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // robot-specific data for generating grasps
  grasp_generator::GraspData grasp_data_;

  // which baxter arm are we using
  std::string arm_;
  std::string ee_group_name_;
  std::string planning_group_name_;

public:

  // Constructor
  GraspGeneratorTest(int num_tests)
    : nh_("~")
  {
    nh_.param("arm", arm_, std::string("left"));
    nh_.param("ee_group_name", ee_group_name_, std::string(arm_ + "_hand"));
    planning_group_name_ = arm_ + "_arm";

    ROS_INFO_STREAM_NAMED("test","Arm: " << arm_);
    ROS_INFO_STREAM_NAMED("test","End Effector: " << ee_group_name_);
    ROS_INFO_STREAM_NAMED("test","Planning Group: " << planning_group_name_);

    // ---------------------------------------------------------------------------------------------
    // Load grasp data specific to our robot
    if (!grasp_data_.loadRobotGraspData(nh_, ee_group_name_))
      ros::shutdown();

    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to Rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base", "/moveit_visual_tools"));

    ros::Duration(1.0).sleep();

    visual_tools_->setLifetime(200.0);
    // visual_tools_->loadMarkerPub();

    // ---------------------------------------------------------------------------------------------
    // Load grasp generator
    simple_grasps_.reset( new grasp_generator::SimpleGrasps(visual_tools_,true) );
    ros::Duration(1.0).sleep();
    // geometry_msgs::Pose pose;
    // visual_tools_->generateEmptyPose(pose);

    // // ---------------------------------------------------------------------------------------------
    // // Animate open and closing end effector

    // for (std::size_t i = 0; i < 4; ++i)
    // {
    //   // Test visualization of end effector in OPEN position
    //   grasp_data_.setRobotStatePreGrasp( visual_tools_->getSharedRobotState() );
    //   visual_tools_->loadEEMarker(grasp_data_.ee_group_);
    //   const robot_model::JointModelGroup* ee_jmg = visual_tools_->getRobotModel()->getJointModelGroup(grasp_data_.ee_group_);
    //   visual_tools_->publishEEMarkers(pose, ee_jmg, rviz_visual_tools::ORANGE, "test_eef");
    //   ros::Duration(1.0).sleep();

    //   // Test visualization of end effector in CLOSED position
    //   grasp_data_.setRobotStateGrasp( visual_tools_->getSharedRobotState() );
    //   visual_tools_->loadEEMarker(grasp_data_.ee_group_);
    //   visual_tools_->publishEEMarkers(pose, ee_jmg, rviz_visual_tools::GREEN, "test_eef");
    //   ros::Duration(1.0).sleep();
    // }

    // ---------------------------------------------------------------------------------------------
    // Generate grasps for a bunch of random objects
    world_model_msgs::Object object;

    std::vector<grasp_generator::Grasp> possible_grasps;

    // Loop
    int i = 0;
    while(ros::ok())
    {
      ROS_INFO_STREAM_NAMED("test","Adding random object " << i+1 << " of " << num_tests);

      // Remove randomness when we are only running one test
      if (num_tests == 1)
        generateTestObject(object);

      // geometry_msgs::Pose object_pose2;
      // generateTestObject(object_pose2);
      // object_pose2.position.x -=0.04;
      // object_pose2.position.y -=0.04;
      // object_pose2.position.z -=0.04;

     
      ROS_INFO_STREAM_NAMED("test", "Publishing object");
      // draw the first object
      if (object.primitives[0].type == object.primitives[0].CYLINDER) {
        visual_tools_->publishCylinder(object.primitive_poses[0], rviz_visual_tools::YELLOW, object.primitives[0].dimensions[0],2*object.primitives[0].dimensions[1]);
        // visual_tools_->publishCylinder(object.primitive_poses[1], rviz_visual_tools::YELLOW, object.primitives[1].dimensions[0],2*object.primitives[1].dimensions[1]);
      }
      else if (object.primitives[0].type == object.primitives[0].BOX) {
        visual_tools_->publishCuboid(object.primitive_poses[0], object.primitives[0].dimensions[0], object.primitives[0].dimensions[1], object.primitives[0].dimensions[2], rviz_visual_tools::YELLOW);
        // visual_tools_->publishCuboid(object.primitive_poses[1], object.primitives[1].dimensions[0], object.primitives[1].dimensions[1], object.primitives[1].dimensions[2], rviz_visual_tools::YELLOW);
      }
      else{
        ROS_ERROR("Cannot publish the FIRST object!");
      }
      ros::Duration(1).sleep();
      // draw the second object
      if (object.primitives[1].type == object.primitives[1].CYLINDER) {
        visual_tools_->publishCylinder(object.primitive_poses[1], rviz_visual_tools::BLUE, object.primitives[1].dimensions[0],2*object.primitives[1].dimensions[1]);
        // visual_tools_->publishCylinder(object.primitive_poses[1], rviz_visual_tools::YELLOW, object.primitives[1].dimensions[0],2*object.primitives[1].dimensions[1]);
      }
      else if (object.primitives[1].type == object.primitives[1].BOX) {
        // visual_tools_->publishCuboid(object.primitive_poses[1], object.primitives[0].dimensions[0], object.primitives[0].dimensions[1], object.primitives[0].dimensions[2], rviz_visual_tools::YELLOW);
        visual_tools_->publishCuboid(object.primitive_poses[1], object.primitives[1].dimensions[0], object.primitives[1].dimensions[1], object.primitives[1].dimensions[2], rviz_visual_tools::BLUE);
      }
      else{
        ROS_ERROR("There is no second object, therefore it cannot be published!");
      }
      ros::Duration(1).sleep();



      possible_grasps.clear();

      // Generate set of grasps for one object
      simple_grasps_->generateBlockGrasps( object, grasp_data_, possible_grasps);
      // Visualize them
      // const robot_model::JointModelGroup* ee_jmg = visual_tools_->getRobotModel()->getJointModelGroup(grasp_data_.ee_group_);
      // visual_tools_->publishAnimatedGrasps(possible_grasps, ee_jmg);
      //visual_tools_->publishGrasps(possible_grasps, grasp_data_.ee_parent_link_);

      // Test if done
      ++i;
      if( i >= num_tests )
        break;
    }
  }

  void generateTestObject(world_model_msgs::Object &object)
  {
    // -----------------cylinder generator---------------
    geometry_msgs::Pose pose;
    shape_msgs::SolidPrimitive shape;
    shape.type = shape.CYLINDER;
    shape.dimensions.resize(2);
    //grasp type 0
    shape.dimensions[shape.CYLINDER_HEIGHT]=0.13;
    shape.dimensions[shape.CYLINDER_RADIUS]=0.08;
    // //grasp type 4
    // shape.dimensions[shape.CYLINDER_HEIGHT]=0.14;
    // shape.dimensions[shape.CYLINDER_RADIUS]=0.08;
    // //grasp type 1
    // shape.dimensions[shape.CYLINDER_HEIGHT]=0.10;
    // shape.dimensions[shape.CYLINDER_RADIUS]=0.06;
    // //grasp type 2
    // shape.dimensions[shape.CYLINDER_HEIGHT]=0.10;
    // shape.dimensions[shape.CYLINDER_RADIUS]=0.04;
    // //grasp type 3
    // shape.dimensions[shape.CYLINDER_HEIGHT]=0.04;
    // shape.dimensions[shape.CYLINDER_RADIUS]=0.04;

    pose.position.x = 0.5;
    pose.position.y = -0.2;
    pose.position.z = 0.5;

    // pose.orientation.x = 0;
    // pose.orientation.y = 0;
    // pose.orientation.z = 0;
    // pose.orientation.w = 1;

    pose.orientation.x = -0.202;
    pose.orientation.y = 0.663;
    pose.orientation.z = 0.116;
    pose.orientation.w = 0.712;

    // Choose which object to test
    object.primitive_poses.push_back(pose);
    object.primitives.push_back(shape);
    object.id = "bowl0";

    //------ add another bowl 
    // shape.type = shape.CYLINDER;
    // shape.dimensions.resize(2);
    // //grasp type 0
    // shape.dimensions[shape.CYLINDER_HEIGHT]=0.13;
    // shape.dimensions[shape.CYLINDER_RADIUS]=0.08;
    // // // //grasp type 4
    // // shape.dimensions[shape.CYLINDER_HEIGHT]=0.14;
    // // shape.dimensions[shape.CYLINDER_RADIUS]=0.08;
    // // // //grasp type 1
    // // shape.dimensions[shape.CYLINDER_HEIGHT]=0.10;
    // // shape.dimensions[shape.CYLINDER_RADIUS]=0.06;
    // // //grasp type 2
    // // shape.dimensions[shape.CYLINDER_HEIGHT]=0.10;
    // // shape.dimensions[shape.CYLINDER_RADIUS]=0.04;
    // // //grasp type 3
    // // shape.dimensions[shape.CYLINDER_HEIGHT]=0.04;
    // // shape.dimensions[shape.CYLINDER_RADIUS]=0.04;
    // pose.position.x = 0;
    // pose.position.y = 0;
    // pose.position.z = 0;
    // pose.orientation.x = -0.202;
    // pose.orientation.y = 0.663;
    // pose.orientation.z = 0.116;
    // pose.orientation.w = 0.712;

    // // pose.orientation.x = 0;
    // // pose.orientation.y = 0;
    // // pose.orientation.z = 0;
    // // pose.orientation.w = 1;

    // // Choose which object to test
    // object.primitive_poses.push_back(pose);
    // object.primitives.push_back(shape);
    // object.id = "bowl1";

    // // ---------------------------------------------
    // -----------------box generator---------------
    geometry_msgs::Pose pose1;
    shape_msgs::SolidPrimitive shape1;
    shape1.type = shape1.BOX;
    shape1.dimensions.resize(3);


    // grasp type 0
    shape1.dimensions[shape1.BOX_X]=0.18;
    shape1.dimensions[shape1.BOX_Y]=0.38;
    shape1.dimensions[shape1.BOX_Z]=0.13;
    // //grasp type 4
    // shape1.dimensions[shape1.BOX_X]=0.18;
    // shape1.dimensions[shape1.BOX_Y]=0.38;
    // shape1.dimensions[shape1.BOX_Z]=0.16;
    // //grasp type 1
    // shape1.dimensions[shape1.BOX_X]=0.18;
    // shape1.dimensions[shape1.BOX_Y]=0.38;
    // shape1.dimensions[shape1.BOX_Z]=0.10;
    // //grasp type 2
    // shape1.dimensions[shape1.BOX_X]=0.09;
    // shape1.dimensions[shape1.BOX_Y]=0.19;
    // shape1.dimensions[shape1.BOX_Z]=0.10;
    // //grasp type 3
    // shape1.dimensions[shape1.BOX_X]=0.18;
    // shape1.dimensions[shape1.BOX_Y]=0.38;
    // shape1.dimensions[shape1.BOX_Z]=0.04;

    pose1.position.x = 0.4;
    pose1.position.y = -0.15;
    pose1.position.z = 0.0;
    // pose1.position.x = 0.0;
    // pose1.position.y = 0.0;
    // pose1.position.z = 0.0;

    // pose1.orientation.x = 0;
    // pose1.orientation.y = 0;
    // pose1.orientation.z = 0;
    // pose1.orientation.w = 1;

    pose1.orientation.x = -0.202;
    pose1.orientation.y = 0.663;
    pose1.orientation.z = 0.116;
    pose1.orientation.w = 0.712;

    // Save the object
    object.primitive_poses.push_back(pose1);
    object.primitives.push_back(shape1);
    object.id = "box0";

    // ------ another box
    // pose.position.x = 0;
    // pose.position.y = 0;
    // pose.position.z = 0;

    // pose.orientation.x = -0.202;
    // pose.orientation.y = 0.663;
    // pose.orientation.z = 0.116;
    // pose.orientation.w = 0.712;

    // pose.orientation.x = 0;
    // pose.orientation.y = 0;
    // pose.orientation.z = 0;
    // pose.orientation.w = 1;

    // Choose which object to test
    // object.primitive_poses.push_back(pose);
    // object.primitives.push_back(shape);
    // object.id = "box1";
  }

  // void generateRandomObject(geometry_msgs::Pose& object_pose)
  // {
  //   // Position
  //   object_pose.position.x = fRand(0.1,0.9); //0.55);
  //   object_pose.position.y = fRand(-0.28,0.28);
  //   object_pose.position.z = 0.02;

  //   // Orientation
  //   double angle = M_PI * fRand(0.1,1);
  //   Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
  //   object_pose.orientation.x = quat.x();
  //   object_pose.orientation.y = quat.y();
  //   object_pose.orientation.z = quat.z();
  //   object_pose.orientation.w = quat.w();
  // }

  // double fRand(double fMin, double fMax)
  // {
  //   double f = (double)rand() / RAND_MAX;
  //   return fMin + f * (fMax - fMin);
  // }

}; // end of class

} // namespace


int main(int argc, char *argv[])
{
  int num_tests = 1;
  ros::init(argc, argv, "grasp_generator_test");

  ROS_INFO_STREAM_NAMED("main","Simple Grasps Test");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Seed random
  srand(ros::Time::now().toSec());


  // Benchmark time
  ros::Time start_time;
  start_time = ros::Time::now();

  // Run Tests
  baxter_pick_place::GraspGeneratorTest tester(num_tests);

  // Benchmark time
  double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
  ROS_INFO_STREAM_NAMED("","Total time: " << duration);
  //std::cout << duration << "\t" << num_tests << std::endl;

  ros::Duration(1.0).sleep(); // let rviz markers finish publishing

  return 0;
}
