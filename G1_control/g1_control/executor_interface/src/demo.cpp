/*
 * =====================================================================================
 *
 *       Filename:  demo.cpp
 *
 *    Description:  Test demo for executor interface
 *
 *        Version:  1.0
 *        Created:  06/16/2016 10:17:43 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ren Mao (neroam), neroam@umd.edu
 *   Organization:  
 *
 * =====================================================================================
 */

#include <g1_control/executor.h>
#include <ros/ros.h>
#include <sstream>
#include <world_model_msgs/GetStatesObjects.h>
#include <world_model_msgs/UpdateStatesObjects.h>
#include <world_model_msgs/QueryLocations.h>

geometry_msgs::Pose current_pose;

bool testGrasp(g1::control::Executor &motion_controller, std::string &side)
{
  g1::control::GraspPtr grasp_ptr(new g1::control::Grasp("Test grasping"));
  grasp_ptr->setGraspCmd(side,
                        g1_control_msgs::G1ControlGraspCodes::CLOSE,
                        0);
  motion_controller.addActionTarget(grasp_ptr);

  std::vector<geometry_msgs::Pose> target_poses;
  geometry_msgs::Pose target_pose_l = motion_controller.getCurrentPose("left_gripper").pose;
  geometry_msgs::Pose target_pose_r = motion_controller.getCurrentPose("right_gripper").pose;

  geometry_msgs::Pose target_pose = (side == "left") ? target_pose_l : target_pose_r;
  target_pose = g1::control::updatePose(target_pose, 0, 0, 0.2);
  target_poses.push_back(target_pose);

  g1::control::MovePtr move_ptr(new g1::control::Move("Lift " + side + " arm"));
  move_ptr->setTargetPoses(side, target_poses);
  motion_controller.addActionTarget(move_ptr);

  grasp_ptr.reset(new g1::control::Grasp("Test openning"));
  grasp_ptr->setGraspCmd(side,
                        g1_control_msgs::G1ControlGraspCodes::OPEN,
                        0);
  motion_controller.addActionTarget(grasp_ptr);

  target_poses.clear();
  target_pose = g1::control::updatePose(target_pose, 0, 0, -0.2);
  target_poses.push_back(target_pose);

  move_ptr.reset(new g1::control::Move("Put down " + side + " arm"));
  move_ptr->setTargetPoses(side, target_poses);
  motion_controller.addActionTarget(move_ptr);

  bool success = motion_controller.run();
  return success;
}

std::vector<world_model_msgs::Object> testWorldModel(ros::ServiceClient &client, bool manipulatable, const std::string &object_id)
{
//   //TODO:: read from world model
//   std::vector<world_model_msgs::Object> objects;
// 
//   //ADD table object
//   world_model_msgs::Object table;
//   table.id = std::string("table");
// 
//   shape_msgs::SolidPrimitive primitive;
//   primitive.type = primitive.BOX;
//   primitive.dimensions.resize(3);
//   primitive.dimensions[0] = 0.8;
//   primitive.dimensions[1] = 1.6;
//   primitive.dimensions[2] = 1.0;
// 
//   /* A pose for the box (specified relative to frame_id) */
//   geometry_msgs::Pose primitive_pose;
//   primitive_pose.orientation.w = 1.0;
//   primitive_pose.position.x = 0.72;
//   primitive_pose.position.y = 0;
//   primitive_pose.position.z = -0.70;
// 
//   table.primitives.push_back(primitive);
//   table.primitive_poses.push_back(primitive_pose);
// 
//   objects.push_back(table);

  // if (object_id == "bowl") {
  // std::vector<world_model_msgs::Object> objects;

  // //ADD target object
  // world_model_msgs::Object bowl;
  // bowl.id = std::string("bowl");

  // shape_msgs::SolidPrimitive primitive;
  // primitive.type = primitive.CYLINDER;
  // primitive.dimensions.resize(2);
  // // Test for type 1
  // // primitive.dimensions[0] = 0.07;
  // // primitive.dimensions[1] = 0.09;  
  // // Test for type 2
  // primitive.dimensions[0] = 0.10;
  // primitive.dimensions[1] = 0.04;
  // // Test for type 0 left
  // // primitive.dimensions[0] = 0.12;
  // // primitive.dimensions[1] = 0.05;
  // // Test for type 0 right
  // // primitive.dimensions[0] = 0.14;
  // // primitive.dimensions[1] = 0.05;
  // // Test for type 3
  // // primitive.dimensions[0] = 0.045;
  // // primitive.dimensions[1] = 0.05;

  // // A pose for the box (specified relative to frame_id) 
  // geometry_msgs::Pose primitive_pose;
  // primitive_pose.orientation.w = 1.0;
  // primitive_pose.position.x = 0.74;
  // primitive_pose.position.y = -0.47;
  // primitive_pose.position.z = -0.15;

  // bowl.primitives.push_back(primitive);
  // bowl.primitive_poses.push_back(primitive_pose);

  // objects.push_back(bowl);

  // return objects;
  // }
  world_model_msgs::GetStatesObjects srv;
  srv.request.object_id = object_id;
  srv.request.manipulatable = manipulatable;

  std::vector<world_model_msgs::Object> objects;
  if (client.call(srv)) {
    objects = srv.response.objects;
  }
  else {
    ROS_ERROR("Failed to call service getting from world model.");
  }

  // if (object_id == "ar_marker_1") {
  //   //ADD target object
  //   objects[0].primitives[0].dimensions[0] = 0.07;
  //   objects[0].primitives[0].dimensions[1] = 0.09;
  // }

  return objects;
}


// bool testPlaceIntoMicrowave(g1::control::Executor &motion_controller, 
//     const world_model_msgs::Object &object, 
//     const world_model_msgs::Object &microwave_door)
// {
//   std::vector<std::string> collision_objects;
//   std::vector<geometry_msgs::Pose> target_poses;
//   geometry_msgs::Pose target_pose;
//   g1::control::MovePtr move_ptr;

//   std::string arm_use("left");

//   // Pick the object
//   g1::control::PickPtr pick_ptr(new g1::control::Pick("Testing Pick the object"));

//   // TODO:: generate grasp planning
//   std::vector<geometry_msgs::Pose> approach_poses, grasp_poses;
//   int grasp_type = 0;
//   if (!testGraspPlanning(object, approach_poses, grasp_poses, grasp_type)) {
//     return false;
//   }
//   collision_objects.push_back(object.id);
//   pick_ptr->setAllowedCollision(collision_objects);
//   pick_ptr->setTargetPicks(arm_use, object, approach_poses, grasp_poses, grasp_type);
//   pick_ptr->setRetries(3);
//   motion_controller.addActionTarget(pick_ptr);

//   // Lift object TODO:: Correct orientation of object??
//   move_ptr.reset(new g1::control::Move("Lift left arm"));
//   target_pose = grasp_poses[0];
//   target_pose = g1::control::updatePose(target_pose, 0, 0, 0.2);
//   target_poses.push_back(target_pose);
//   move_ptr->setTargetPoses(arm_use, target_poses);
//   move_ptr->setAllowedCollision(collision_objects);
//   move_ptr->setLockOrientation(arm_use, 0.1, 0.1, 1000);
//   motion_controller.addActionTarget(move_ptr);

//   // Place to the front of microwave
//   move_ptr.reset(new g1::control::Move("Move to front of microwave"));
//   geometry_msgs::Pose object_pose = microwave_door.primitive_poses[0];
//   Eigen::Quaternionf object_quat(object_pose.orientation.w,
//       object_pose.orientation.x,
//       object_pose.orientation.y,
//       object_pose.orientation.z);
//   Eigen::Vector3f target_z = -1.0 * object_quat.matrix().col(0);
//   Eigen::Quaternionf hand_quat(target_pose.orientation.w,
//       target_pose.orientation.x,
//       target_pose.orientation.y,
//       target_pose.orientation.z);
//   Eigen::Vector3f hand_z = hand_quat.matrix().col(2);
//   hand_quat = hand_quat * Eigen::Quaternionf::FromTwoVectors(hand_z, target_z);
//   target_pose = g1::control::updatePose(object_pose, -0.15, 0, 0.08);
//   target_pose.orientation.x = hand_quat.x();
//   target_pose.orientation.y = hand_quat.y();
//   target_pose.orientation.z = hand_quat.z();
//   target_pose.orientation.w = hand_quat.w();
//   target_poses.clear();
//   target_poses.push_back(target_pose);
//   move_ptr->setRetries(3);
//   collision_objects.push_back("microwave_frame");
//   collision_objects.push_back("microwave_door");
//   move_ptr->setAllowedCollision(collision_objects);
//   move_ptr->setTargetPoses(arm_use, target_poses);
//   move_ptr->setLockOrientation(arm_use, 0.1, 0.1, 1000);
//   motion_controller.addActionTarget(move_ptr);

//   // Place into the microwave
//   // Place planning TODO: should we do that elegent?
//   std::vector<geometry_msgs::Pose> place_poses, retreat_poses;
//   target_pose = g1::control::updatePose(target_pose, 0.10, 0, 0);
//   place_poses.push_back(target_pose);
//   target_pose = g1::control::updatePose(target_pose, -0.14, 0, 0.01);
//   retreat_poses.push_back(target_pose);
//   grasp_type = 1;

//   g1::control::PlacePtr place_ptr(new g1::control::Place("Testing Place into microwave"));
//   place_ptr->setTargetPlaces(arm_use, object, place_poses, retreat_poses, grasp_type);
//   place_ptr->setAllowedCollision(collision_objects);
//   place_ptr->setRetries(3);
//   motion_controller.addActionTarget(place_ptr);

//   // Retreating hand
//   target_pose = g1::control::updatePose(target_pose, -0.2, 0, 0);
//   target_poses.clear();
//   target_poses.push_back(target_pose);
//   move_ptr.reset(new g1::control::Move("retreat left arm"));
//   move_ptr->setTargetPoses(arm_use, target_poses);
//   // move_ptr->setLockPosition(0.001, 0.001, 0.2);
//   move_ptr->setRetries(3);
//   motion_controller.addActionTarget(move_ptr);

//   // target_pose = g1::control::updatePose(target_pose, 0, -0.1, 0);
//   // target_poses.clear();
//   // target_poses.push_back(target_pose);
//   // move_ptr.reset(new g1::control::Move("retreat left arm"));
//   // move_ptr->setTargetPoses(arm_use, target_poses);
//   // // move_ptr->setLockPosition(0.001, 0.001, 0.2);
//   // // move_ptr->setLockOrientation();
//   // motion_controller.addActionTarget(move_ptr);

//   // target_pose = g1::control::updatePose(target_pose, -0.1, 0, 0);
//   // target_poses.clear();
//   // target_poses.push_back(target_pose);
//   // move_ptr.reset(new g1::control::Move("retreat left arm"));
//   // move_ptr->setTargetPoses(arm_use, target_poses);
//   // move_ptr->setRetries(3);
//   // // move_ptr->setLockPosition(0.001, 0.001, 0.2);
//   // // move_ptr->setLockOrientation();
//   // motion_controller.addActionTarget(move_ptr);

//   return motion_controller.run();
// }


bool testPushButton(g1::control::Executor &motion_controller)
{
  geometry_msgs::Pose button_pose, door_pose;
  tf::TransformListener tf_listener;

  // if (!g1::control::getPosefromTF(button_pose,tf_listener, "base", "start_button")) {
  //   return false;
  // }

  if (!g1::control::getPosefromTF(door_pose,tf_listener, "base", "world_model/microwave_door")) {
    return false;
  }

  button_pose = door_pose;

  std::vector<geometry_msgs::Pose> target_poses;
  geometry_msgs::Pose target_pose = g1::control::updatePose(button_pose, -0.13, -0.055, 0);
  target_pose.orientation = door_pose.orientation;
  target_pose = g1::control::rotatePose(target_pose, 3.14, 1.57, 0);
  target_poses.push_back(target_pose);

  g1::control::MovePtr move_ptr(new g1::control::Move("Align gripper"));
  move_ptr->setTargetPoses("right", target_poses);
  motion_controller.addActionTarget(move_ptr);

  move_ptr.reset(new g1::control::Move("Push the button"));
  target_pose = g1::control::updatePose(target_pose, 0.05, 0, 0);
  target_poses.clear();
  target_poses.push_back(target_pose);
  move_ptr->setLockPosition("right", 0.005, 0.005, 0.06);
  std::vector<std::string> collision_objects;
  collision_objects.push_back("microwave_door");
  collision_objects.push_back("microwave_frame");
  collision_objects.push_back("microwave_handle");
  move_ptr->setAllowedCollision(collision_objects);
  move_ptr->setTargetPoses("right", target_poses);
  motion_controller.addActionTarget(move_ptr);

  move_ptr.reset(new g1::control::Move("Retreating the hand"));
  target_poses.clear();
  target_pose = g1::control::updatePose(target_pose, -0.10, 0, 0);
  target_poses.push_back(target_pose);
  move_ptr->setLockPosition("right", 0.005, 0.005, -0.11);
  // move_ptr->setLockOrientation();
  move_ptr->setTargetPoses("right", target_poses);
  move_ptr->setRetries(5);
  motion_controller.addActionTarget(move_ptr);

  return motion_controller.run();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "g1_control_executor_demo", ros::init_options::AnonymousName);
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::ServiceClient world_model_client = node_handle.serviceClient<world_model_msgs::GetStatesObjects>("world_model/get_states_objects");
  ros::ServiceClient world_model_client_update = node_handle.serviceClient<world_model_msgs::UpdateStatesObjects>("world_model/update_states_objects");
  ros::ServiceClient location_service = node_handle.serviceClient<world_model_msgs::QueryLocations>("world_model/query_locations");

  g1::control::Executor::Options opt(node_handle, "right", "electric", "reflex");
  g1::control::Executor motion_controller(opt);

  std::string arm_use = "right";

  motion_controller.resetArms(arm_use);
  // motion_controller.calibrateGrippers(arm_use);
  motion_controller.initGrippers(arm_use);
  motion_controller.clearPlanningScene();


  //========================
  // Read Information from world model
  //=========================

  std::vector<world_model_msgs::Object> objects = testWorldModel(world_model_client, false, "");
  motion_controller.updatePlanningScene(objects);

  objects = testWorldModel(world_model_client, true, "");
  motion_controller.updatePlanningScene(objects);

  return 0;
  // objects = testWorldModel(world_model_client, false, "microwave_frame");
  // for (int i = 0; i < objects[0].primitive_poses.size(); i ++) {
  //   objects[0].primitive_poses[i] = g1::control::updatePose(objects[0].primitive_poses[i], 0.10, 1.02, 0);
  // }
  // motion_controller.updatePlanningScene(objects);

  // objects = testWorldModel(world_model_client, true, "microwave_door");
  // world_model_msgs::Object microwave_door = objects[0];
  // microwave_door.primitive_poses[0] = g1::control::updatePose(microwave_door.primitive_poses[0], 0.10, 1.02, 0);

  // objects[0].primitive_poses[0] = g1::control::updatePose(objects[0].primitive_poses[0], 
  //     0.10 -objects[0].primitives[0].dimensions[1]/2.0, 
  //     1.02 + objects[0].primitives[0].dimensions[1]/2.0,
  //     0);
  // objects[0].primitive_poses[0] = g1::control::rotatePose(objects[0].primitive_poses[0], 0, 0, -1.57);
  // motion_controller.updatePlanningScene(objects);

  // int idx = 4;
  // for (int i = 0; i < idx; i++) {
  //   std::ostringstream ss;
  //   ss << i;
  //   objects = testWorldModel(world_model_client, true, "dish_" + ss.str());
  //   motion_controller.updatePlanningScene(objects);
  // }

  
  bool success = false;

  // =============================
  // Read parameter server
  // =============================
  std::string object_id = "dish_0";
  std::string place_location = "";

  if (!node_handle.hasParam("/control_demo/pick_object")) {
    ROS_WARN("Use default object id as target");
  }
  node_handle.getParam("/control_demo/pick_object", object_id);

  std::vector<double> target_offset;
  if (!node_handle.hasParam("/control_demo/pick_object_offset")) {
    ROS_WARN("Use default offset as target");
    target_offset.resize(3, 0.0);
  }
  node_handle.getParam("/control_demo/pick_object_offset", target_offset);

  if (!node_handle.hasParam("/control_demo/place_location")) {
    ROS_WARN("Use default place location as target");
  }
  node_handle.getParam("/control_demo/place_location", place_location);

  // success = testGrasp(motion_controller, arm_use);

  // objects = testWorldModel(world_model_client, true);
  // motion_controller.updatePlanningScene(objects);

  // success = testPushButton(motion_controller);

  // objects = testWorldModel(world_model_client, true, "dish_0");

  objects = testWorldModel(world_model_client, true, object_id);
  motion_controller.updatePlanningScene(objects);
  // ===============================
  // Test pick up object 
  // ===============================
  world_model_msgs::Object target_object = objects[0];

  ROS_INFO_STREAM(target_object);
  int grasp_type = -1;
  bool enable_vertical = true;
  success = motion_controller.pickObject(arm_use, target_object, grasp_type, enable_vertical);
  if (success) { 
    // Pick Plan Succeed
    success = motion_controller.run();
  } 
  else { 
    ROS_WARN("Cannot grasp object via horizontal side. Trying with vertical approachs.");
    success = motion_controller.pickObject(arm_use, target_object, grasp_type, true);
    // Plan failed
    // TODO:: Fallback to rerun detection and planning
  }

  if (!success) { 
    // Pick Execution Failed
    ROS_ERROR("Cannot pick up object");
    return 0;    
  }
  

  // ===============================
  // Test place object 
  // ===============================

  geometry_msgs::Point target_location = g1::control::updatePose(
      target_object.primitive_poses[0], 
      target_offset[0], 
      target_offset[1], 
      target_offset[2]).position;
  
  if (place_location != "") {
    world_model_msgs::QueryLocations srv;
    srv.request.operation = srv.request.READ;
    srv.request.location_id = place_location;

    if (location_service.call(srv)) {
      if (srv.response.locations.size() > 0) {
        target_location = srv.response.locations[0].position;

        double height = 0;
        if (target_object.primitives[0].type == target_object.primitives[0].BOX) {
          height = target_object.primitives[0].dimensions[2];
        }
        else if (target_object.primitives[0].type == target_object.primitives[0].CYLINDER) {
          height = target_object.primitives[0].dimensions[0];
        }

        target_location.z += height / 2.0;

        ROS_INFO_STREAM(target_location);
        ROS_INFO("Get location from world model");
      }
    }
    else {
      ROS_ERROR("Failed to call location service getting from world model.");
    }
  }

  success = motion_controller.placeObject(arm_use, target_location, target_object, grasp_type);

  if (success) { 
    // Place Plan Succeed
    success = motion_controller.run();
  } 
  else { 
    // Plan failed
    // TODO:: Fallback to rerun detection and planning

    // Plan to place to the original location
    ROS_WARN_STREAM("Failed to plan place object, put into original location");
    motion_controller.reset();
    success = motion_controller.placeObject(arm_use, target_object.primitive_poses[0].position, target_object, grasp_type);
    if (success) {
      motion_controller.run();
    }
  }

  if (!success) { 
    // Place Execution Failed
    ROS_ERROR("Cannot place object");
    return 0; 
  }
  
  // ===============================
  // Test update world model 
  // ===============================
  motion_controller.updateObjectInfo(target_object);
  std::vector<world_model_msgs::Object> objects_info(1, target_object);
  
  world_model_msgs::UpdateStatesObjects srv;
  srv.request.objects_info = objects_info;
  srv.request.operation = srv.request.UPDATE;

  if (world_model_client_update.call(srv)) {
    ROS_INFO("Updated world model");
  }
  else {
    ROS_ERROR("Failed to call service getting from world model.");
  }


  // current_pose = motion_controller.getCurrentPose("left_gripper").pose;
  // success = testPlaceIntoMicrowave(motion_controller, objects[0], microwave_door);

  // success = testPick(motion_controller, objects[0]);

  // objects[0].primitive_poses[0] = g1::control::updatePose(objects[0].primitive_poses[0], 0.2, 0, 0);
  // success = testPlace(motion_controller, objects[0]);


  if (success) {
    // motion_controller.resetArms(arm_use);
    // motion_controller.resetGrippers(arm_use); 
    ROS_INFO("Demo accomplished!");
  }

  return 0;
}


