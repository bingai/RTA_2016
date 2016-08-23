#include <dummy_vision/dummy_vision.h>

namespace g1
{

namespace vision
{

DummyVision::DummyVision(ros::NodeHandle &node_handle) 
  : str_("Empty")
{
  update_client_ = node_handle.serviceClient<world_model_msgs::UpdateStatesDummy>("/world_model/update_states_dummy");
  if (!update_client_.waitForExistence(ros::Duration(5))) {
    ROS_ERROR("Cannot connect to the service");
  }
  ROS_INFO("Dummy vision initialized.");
}

DummyVision::~DummyVision() 
{}

// Core function to run processing
void DummyVision::run(const std::string & str1, const std::string & str2) {
	str_ = str1 + str2;
	ROS_INFO("run done!");
}

bool DummyVision::updateWorldStates()
{
  world_model_msgs::UpdateStatesDummy srv;
  srv.request.dummy_msg = str_;
  if (update_client_.call(srv))
  {
    ROS_INFO_STREAM("Update World Model Success: " << str_ );
  }
  else {
    ROS_ERROR("Failed to update world model.");
    return false;
  }
  return true;
}


}
}
