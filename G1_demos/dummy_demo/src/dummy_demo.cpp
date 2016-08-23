#include <dummy_demo/dummy_demo.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "g1_dummy_demo", ros::init_options::AnonymousName);

  ros::NodeHandle node_handle;

  g1::vision::DummyVision dv(node_handle);

//   Process the vision
  dv.run("Dummy", "Vision Test");

//   Update world states
  if (dv.updateWorldStates()) {
    ROS_INFO("Updated world states.");
  }
  ros::Duration(2).sleep();

//   Process and update
  dv.run("Dummy", "Vision Test2");
  if (dv.updateWorldStates()) {
    ROS_INFO("Demo finished.");
  }

  return 0;
};

