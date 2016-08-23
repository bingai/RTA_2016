#ifndef G1_WORLD_MODEL_
#define G1_WORLD_MODEL_

#include <std_msgs/String.h>
#include <ros/ros.h>
namespace g1
{

namespace common
{

class WorldModel
{
public:
  WorldModel(ros::NodeHandle &node_handle);

  ~WorldModel();

  void initializeScene();

  void publishStates();

  void updateStates();

private:
  class WorldModelImpl;
  WorldModelImpl *impl_;
  
};


}

}

#endif

