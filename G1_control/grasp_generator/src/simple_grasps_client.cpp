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
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>

// Grasp generation
// #include <grasp_generator/simple_grasps.h>
#include <grasp_generator/GenerateGraspsAction.h>
#include <boost/thread.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
// #include <eigen_conversions/eigen_msg.h>

#include <world_model_msgs/GetStatesObjects.h>

// Baxter specific properties
// #include <grasp_generator/grasp_data.h>
// #include <grasp_generator/grasp_filter.h>
// #include <grasp_generator/custom_environment2.h>

void spinThread(){
  ros::spin();
}

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

void generateRandomObject(world_model_msgs::Object &object)
{
  // Tool for visualizing things in Rviz
  // moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // Table data
  static const double TABLE_HEIGHT = 1;
  static const double TABLE_WIDTH = 1.6;
  static const double TABLE_DEPTH = .8;
  static const double TABLE_X = 0.7;
  static const double TABLE_Y = 0;
  static const double TABLE_Z = -0.7;

  // Position
  geometry_msgs::Pose pose;
  shape_msgs::SolidPrimitive shape;

  shape.type = shape.CYLINDER;
  shape.dimensions.resize(2);
  // type 1
  shape.dimensions[shape.CYLINDER_HEIGHT]=0.08;
  shape.dimensions[shape.CYLINDER_RADIUS]=0.08;

  pose.position.x = 1.00;
  pose.position.y = -0.27;
  pose.position.z = -0.15;

  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  //type 2
  // shape.dimensions[shape.CYLINDER_HEIGHT]=0.08;
  // shape.dimensions[shape.CYLINDER_RADIUS]=0.04;


  // pose.position.x = fRand( TABLE_X - TABLE_DEPTH/2.0 , TABLE_X);
  // pose.position.y = fRand( TABLE_Y  , TABLE_Y + TABLE_WIDTH/2.0 );
  // pose.position.z = TABLE_Z + TABLE_HEIGHT / 2.0 +  shape.dimensions[shape.CYLINDER_HEIGHT] /2.0 + 0.2;

  // double angle = M_PI * fRand(0.1, 1.0);
  // Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
  // pose.orientation.x = quat.x();
  // pose.orientation.y = quat.y();
  // pose.orientation.z = quat.z();
  // pose.orientation.w = quat.w();

  // Choose which object to test
  object.primitive_poses.push_back(pose);
  object.primitives.push_back(shape);
  object.id = "bowl";
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "grasp_generator_client");

  // create the action client
  actionlib::SimpleActionClient<grasp_generator::GenerateGraspsAction> client("grasp_generator_server/reflex");
  boost::thread spin_thread(&spinThread);

  ROS_INFO("Waiting for action server to start.");
  client.waitForServer();

  ROS_INFO("Action server started, sending goal.");
  grasp_generator::GenerateGraspsGoal goal;

  // // get object from world model
  // ros::NodeHandle n;
  // ros::ServiceClient client_get_object = n.serviceClient<world_model_msgs::GetStatesObjects>("/world_model/get_states_objects");
  // world_model_msgs::GetStatesObjects srv;
  // // srv.request.object_id = object_id;
  // srv.request.object_id = "dish_0";
  // srv.request.manipulatable = true;

  // std::vector<world_model_msgs::Object> objects;
  // if(client_get_object.call(srv)){
  //   ROS_INFO("GRSEEEEEE!");
  //   objects = srv.response.objects;
  // }
  // else{
  //   ROS_ERROR("Failed to call service getting from world model.");
  // }

  // goal.object = objects[0];
  // ROS_INFO_STREAM_NAMED("DEBUG", "-------goal.object: ---------" << objects[0].primitives[0].dimensions[0] << objects[0].primitives[0].dimensions[1]);
  // ROS_INFO_STREAM_NAMED("DEBUG", "-------goal.object: ---------" << goal.object.primitives[0].dimensions[0] << goal.object.primitives[0].dimensions[1]);


  generateRandomObject(goal.object);
  client.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = client.waitForResult(ros::Duration(5.0));

  if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = client.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
      std::vector<grasp_generator::Grasp> result01 = client.getResult()->grasps;
      // ROS_INFO_STREAM_NAMED("test", "result01 "<< result01[0].grasp_pose);
      // std::vector<grasp_generator::Grasp> result02 = (*client.getResult()).grasps;
      // ROS_INFO_STREAM_NAMED("test", "result02 "<< result02[0].grasp_pose);
   }
    else
      ROS_INFO("Action did not finish before the time out.");

    // shutdown the node and join the thread back before exiting
    ros::shutdown();
    spin_thread.join();

    //exit
    return 0;
}
