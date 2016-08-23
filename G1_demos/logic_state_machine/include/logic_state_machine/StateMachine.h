#ifndef LOGIC_STATE_MACHINE_H
#define LOGIC_STATE_MACHINE_H

#include <ros/ros.h>
#include <stdio.h>
#include <world_model_msgs/exec_state.h>
#include <world_model_msgs/cmd.h>
#include <world_model_msgs/ProgramOrder.h>


#include <g1_control/executor.h>
#include <world_model_msgs/Location.h>

#include <microwave_detection/mircowave_detect.h>

namespace g1
{
namespace common {

class LogicState {
  private:
  	ros::ServiceServer state_update_service_;
  	ros::ServiceServer state_cmd_service_;
    ros::ServiceServer program_order_service_;
    ros::ServiceClient location_client_;

    // World model objects service
    ros::ServiceClient objects_get_client_;
    ros::ServiceClient objects_update_client_;

    // Service for dish detection
    ros::ServiceClient objects_detection_client_;

    // Executor
    g1::control::Executor motion_controller_;

  	int current_state_;
    ros::NodeHandle node_handle_;


  	world_model_msgs::Location target_location_;
  	std::string target_object_;

  	bool updateCurrentStates(world_model_msgs::exec_state::Request &req,
  	  world_model_msgs::exec_state::Response &res);
    bool parseCommand(world_model_msgs::cmd::Request &req,
      world_model_msgs::cmd::Response &res);
    bool learnProgram(world_model_msgs::ProgramOrder::Request& req,
      world_model_msgs::ProgramOrder::Response &res);

    std::vector<world_model_msgs::Object> readWorldModel(const std::string &object_id, bool manipulatable);
    void moveObject(const std::string &object_id, const world_model_msgs::Location &location);
    void reset(); 
    void moveToMicrowave(const std::string &object_id); 
	  void openMicrowave();
	  void closeMicrowave();
    void detectMicrowave();
    void checkObjectidExist();
    void detectObjects();
    void moveFromMicrowave(const std::string &object_id, const world_model_msgs::Location &location);


  public:
  	LogicState(ros::NodeHandle &node_handle);
  	~LogicState();
  	void setState(int state);
  	void run(); 


};

}

}

#endif
