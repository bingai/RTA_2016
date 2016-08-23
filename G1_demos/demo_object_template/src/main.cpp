
// ROS includes
#include <ros/ros.h>

#include <world_model_msgs/GetStatesObjects.h>
#include <world_model_msgs/UpdateStatesObjects.h>
#include <world_model_msgs/Object.h>
#include <geometry_msgs/Pose.h>
#include <shape_msgs/SolidPrimitive.h>
#include <g1_control/executor.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_object_template", ros::init_options::AnonymousName);
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Service to get and update world model objects information
  ros::ServiceClient world_model_get = node_handle.serviceClient<world_model_msgs::GetStatesObjects>("world_model/get_states_objects");
  ros::ServiceClient world_model_update = node_handle.serviceClient<world_model_msgs::UpdateStatesObjects>("world_model/update_states_objects");

  // Fake perception to get objects info
  // Check SolidPrimitive.msg for detail description of primitive shapes and dimensions
  shape_msgs::SolidPrimitive sp;
  sp.type = sp.BOX;
  sp.dimensions.push_back(1.0);
  sp.dimensions.push_back(1.0);
  sp.dimensions.push_back(1.0);

  geometry_msgs::Pose primitive_pose;
  primitive_pose.position = g1::control::constructPoint(0.5, 0.5, 0);
  primitive_pose.orientation = g1::control::constructQuat(0, 0, 0, 1.0);

  // Generate object message
  // Check Object.msg for detail description
  world_model_msgs::Object fake_object;
  fake_object.id = "fake_object";
  fake_object.primitives.push_back(sp);
  fake_object.primitive_poses.push_back(primitive_pose);
  
  // Generate request to update world model
  // Check UpdateStatesObjects.srv for detail 
  world_model_msgs::UpdateStatesObjects srv;
  srv.request.objects_info.push_back(fake_object);

  if (world_model_update.call(srv)) {
    ROS_INFO("Update world model successfully!");
  }
  else {
    ROS_ERROR("Failed to call service getting from world model.");
  }
  
  //-------------------------------------------------------------
  // read from world model
  //-------------------------------------------------------------
  world_model_msgs::GetStatesObjects update_srv;
  update_srv.request.object_id = "fake_object";
  update_srv.request.manipulatable = false;

  std::vector<world_model_msgs::Object> objects;
  if (world_model_get.call(update_srv)) {
    objects = update_srv.response.objects;
  }
  else {
    ROS_ERROR("Failed to call service getting from world model.");
  }
  
  for (int i = 0 ; i < objects.size(); i++) {
    ROS_INFO_STREAM("===================");
    ROS_INFO_STREAM("Object info : " << i);
    ROS_INFO_STREAM(objects[i]);
  }

  
  return 0;
}

