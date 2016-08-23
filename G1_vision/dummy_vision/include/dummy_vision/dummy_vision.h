#ifndef DUMMY_VISION_H
#define DUMMY_VISION_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <world_model_msgs/UpdateStatesDummy.h>

namespace g1 
{

namespace vision 
{

class DummyVision 
{
public:

  DummyVision(ros::NodeHandle &node_handle);
  ~DummyVision();
  void run(const std::string & str1, const std::string & str2);
  bool updateWorldStates();

private:
  std::string str_;
  ros::ServiceClient update_client_;

};

}
}
#endif
