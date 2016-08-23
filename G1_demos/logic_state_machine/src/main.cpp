// ROS includes
#include <ros/ros.h>

// state machine
#include <logic_state_machine/StateMachine.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_machine_main", ros::init_options::AnonymousName);
  ros::NodeHandle node_handle;
  
  ros::AsyncSpinner spinner(2);
  spinner.start();
  
  ros::Rate loop_rate(10);
  g1::common::LogicState state_machine(node_handle);

  while (ros::ok()) {
    state_machine.run();
    loop_rate.sleep();
  }
 
}

