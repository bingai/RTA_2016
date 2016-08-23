#include <logic_state_machine/StateMachine.h>
#include <world_model_msgs/GetStatesObjects.h>
#include <world_model_msgs/UpdateStatesObjects.h>
#include <world_model_msgs/QueryLocations.h>
#include <reconstruction/reconstruction.hpp>
#include <object_detection_3d_node/ObjectDetection3dService.h>


namespace g1
{
namespace common {

LogicState::LogicState(ros::NodeHandle &node_handle)
	: node_handle_(node_handle),
   current_state_(0),
	 motion_controller_(g1::control::Executor::Options(node_handle, "both", "electric", "reflex"))
{
  state_update_service_ = node_handle.advertiseService("/state_machine/current_state", &LogicState::updateCurrentStates, this);
  state_cmd_service_ = node_handle.advertiseService("/state_machine/cmd", &LogicState::parseCommand, this);
  program_order_service_ = node_handle.advertiseService("/state_machine/program_order", &LogicState::learnProgram, this);
  
  location_client_ = node_handle.serviceClient<world_model_msgs::QueryLocations>("world_model/query_locations");

  if (!location_client_.waitForExistence(ros::Duration(2))) {
    ROS_ERROR("Cannot connect to the location service");
  }

  objects_get_client_ = node_handle.serviceClient<world_model_msgs::GetStatesObjects>("world_model/get_states_objects");
  objects_update_client_= node_handle.serviceClient<world_model_msgs::UpdateStatesObjects>("world_model/update_states_objects");
  if (!objects_get_client_.waitForExistence(ros::Duration(2))) {
    ROS_ERROR("Cannot connect to the objects service");
  }


  objects_detection_client_ = node_handle.serviceClient<object_detection_3d_node::ObjectDetection3dService>("object_detection_3d_service");
  if (!objects_detection_client_.waitForExistence(ros::Duration(2))) {
    ROS_ERROR("Cannot connect to the objects detection service");
  }

  reset(); 
}

LogicState::~LogicState() {
	
}

bool LogicState::learnProgram(world_model_msgs::ProgramOrder::Request& req,
  world_model_msgs::ProgramOrder::Response &res) {
  // if (ros::param::has(req.task_name) )
  std::vector<std::string> ne_ids;
  
  std::string file_path;
  // std::string file_path = "/home/arclab/arc_ws/src/g1_demos/logic_state_machine/launch/program.yaml";
  if (!ros::param::get("/state_machine_demo/file_path", file_path)) {
    std::cout << file_path << std::endl;
    ROS_ERROR("Cannot save file path for program order");
    return false;
  }
  
  for (int i = 1; i < req.steps.size() - 1; i++) {
  // Parse location field and query from world model  
    world_model_msgs::QueryLocations srv;
    srv.request.location_id = req.steps[i];
    srv.request.operation = srv.request.READ;

    if (location_client_.call(srv))
    {
      if (srv.response.success) {
        if (srv.response.locations.size() == 0) {
          ROS_INFO_STREAM("No such location defined");
          ne_ids.push_back(req.steps[i]);
        } else {
          ROS_INFO_STREAM("Got location");
        }
      } else {
        ROS_INFO_STREAM("world_model location service error");
      }

    }
    else {
      ROS_INFO_STREAM("\n\nWorld model server down\n\n");
    }
    
  }

  res.nonexist_ids = ne_ids;
  if (ne_ids.size() > 0) {
    ROS_ERROR("Some locations do not exist!");
    return true;
  }
  std::string paramKey = "/state_machine_demo/program_order/" + req.task_name;
  ros::param::set(paramKey, req.steps);
  
  std::string cmd = "rosparam dump " + file_path + " /state_machine_demo/program_order";
  system(cmd.c_str());


  return true;
}

bool LogicState::updateCurrentStates(world_model_msgs::exec_state::Request &req,
  world_model_msgs::exec_state::Response &res) {
  
  // std::cout << "Receive request " << req.get_state << std::endl;
  res.curr_state = current_state_;
  return true;
}


bool LogicState::parseCommand(world_model_msgs::cmd::Request &req,
  world_model_msgs::cmd::Response &res) {

  if (req.action == "reset") {
    setState(5);
    res.cmd_state = 1;
    return true;
  } 
  else if (req.action == "reheat") {
    target_object_ = req.utensil;
    // setState(6);
    setState(21);
    res.cmd_state = 1;
    return true;
  }
  else if (req.action == "open") {
    setState(10);
    res.cmd_state = 1;
    return true;
  } 
  else if (req.action == "close") {
    setState(11);
    res.cmd_state = 1;
    return true;
  } 
  else if (req.action == "detectM") {
    setState(21);
    res.cmd_state = 1;
    return true;
  }  
  else if (req.action == "click") {
    setState(34);
    target_object_ = req.utensil;
    res.cmd_state = 1;
    return true;
  }
  else if (req.action == "detectObjects") {
    setState(40);
    res.cmd_state = 1;
    return true;
  }
  


  // Parse location field and query from world model  
  world_model_msgs::QueryLocations srv;
  srv.request.location_id = req.location_id;
  srv.request.operation = srv.request.READ;

  if (location_client_.call(srv))
  {
    if (srv.response.success) {
      if (srv.response.locations.size() == 0) {
        ROS_INFO_STREAM("No such location defined");
        res.cmd_state = 0;
        return true;
      } else {
        ROS_INFO_STREAM("Got location");
      	res.cmd_state = 1;
      }
    } else {
      ROS_INFO_STREAM("world_model location service error");
      res.cmd_state = -2;
      return false;
    }

  }
  else {
  	ROS_INFO_STREAM("\n\nWorld model server down\n\n");
  	res.cmd_state = -1;
  	return false;
  }


  // parse utensil field
  if (req.utensil == "") {
  	ROS_ERROR_STREAM("Unknow object in received command");
  	res.cmd_state = -1;
  	return false;
  }

  // Parse action field
  if (req.action == "move" || req.action == "put") {
  	target_location_ = srv.response.locations[0];
  	target_object_ = req.utensil;
  	setState(1);
  }
  else if (req.action == "pickMicrowave") {
    setState(50);
    res.cmd_state = 1;
    target_object_ = req.utensil;
    target_location_ = srv.response.locations[0];
    return true;
  }
  else {
  	ROS_ERROR_STREAM("Unknow action in received command");
  	res.cmd_state = -1;
  	return false;
  }

  return true;
}


void LogicState::setState(int state) {
  current_state_ = state;
}

void LogicState::run() 
{
	switch (current_state_) {
		case 0:
			break;
		case 1: // Start to run move object task
			moveObject(target_object_, target_location_);
      break;
    case 5: // Reset to idle
      reset();
      break;
    case 6: // Start to pick and place into microwave
      moveToMicrowave(target_object_);
      break;
    case 10: // Test to open microwave
      openMicrowave();
      break;
    case 11: // Test to close microwave
      closeMicrowave();
      break;
    case 21:
      detectMicrowave();
      break;
    case 34:
      checkObjectidExist();
      break;
    case 40:
      detectObjects();
      break;
    case 50:
      moveFromMicrowave(target_object_, target_location_);
      break;
		default:
			break;
	}

}

void LogicState::reset() 
{
  std::string arm_use = "both";
  motion_controller_.resetArms(arm_use);
  motion_controller_.initGrippers(arm_use);
  ROS_INFO_STREAM("Waiting for new command");
  setState(0);
}

std::vector<geometry_msgs::Pose> getDemoPoses() {
  std::vector<geometry_msgs::Pose> poses(0);

  // poses.push_back(g1::control::constructPose(
  //   g1::control::constructPoint(0.608, -0.125, 0.067),
  //   g1::control::constructQuat(0.471, 0.764, -0.132, 0.420)
  // )); 

  // poses.push_back(g1::control::constructPose(
  //   g1::control::constructPoint(0.616, 0.043, 0.311),
  //   g1::control::constructQuat(0.587, 0.706, -0.212, 0.336)
  // ));

  // poses.push_back(g1::control::constructPose(
  //   g1::control::constructPoint(0.729, -0.202, 0.475),
  //   g1::control::constructQuat(0.645, 0.734, -0.087, 0.196)
  // ));

  poses.push_back(g1::control::constructPose(
    g1::control::constructPoint(0.692, 0.653, 0.335),
    g1::control::constructQuat(-0.037, 0.695, -0.048, 0.717)
  ));

  poses.push_back(g1::control::constructPose(
    g1::control::constructPoint(0.681, -0.024, 0.416),
    g1::control::constructQuat(0.564, 0.726, -0.267, 0.289)
  )); 

  poses.push_back(g1::control::constructPose(
    g1::control::constructPoint(0.584, 0.027, 0.270),
    g1::control::constructQuat(0.459, 0.756, -0.283, 0.371)
  ));

  poses.push_back(g1::control::constructPose(
    g1::control::constructPoint(0.692, 0.653, 0.335),
    g1::control::constructQuat(-0.037, 0.695, -0.048, 0.717)
  ));


  return poses;
}

void LogicState::detectObjects()
{
  
  std::string scene_name = "table_top";

  //========================
  // Read Information from world model
  //=========================
  std::vector<world_model_msgs::Object> objects = readWorldModel("", false);
  motion_controller_.updatePlanningScene(objects);

  objects = readWorldModel("", true);
  motion_controller_.updatePlanningScene(objects);

  std::string arm_use = "left";
  motion_controller_.resetArms(arm_use);

  ROS_INFO_STREAM("Reconstructing the scene on the table");
  Reconstruction rec(node_handle_, &motion_controller_);

  rec.run(getDemoPoses(), scene_name);
  rec.updateWorldState();
  ROS_INFO("Updated reconstructed scene in world states.");

  motion_controller_.resetArms(arm_use);

  ROS_INFO_STREAM("Finished task of scene reconstruction");
 
  ROS_INFO_STREAM("Detecting the objects on the table");
  object_detection_3d_node::ObjectDetection3dService srv;
  srv.request.reconstruction_id = scene_name;
  // Send request
  if (!objects_detection_client_.call (srv))
  {
    ROS_ERROR("Failed to get response. Did the server node die?");
  }
  else
  {
    ROS_INFO("Request sent, response received");
    ROS_INFO("Found %d objects", static_cast<int>(srv.response.objects.size()));
  }

  setState(34);
}

void LogicState::checkObjectidExist() {
  
  bool findOject = false;
  bool firstTime = true;
  while (!findOject) {
    world_model_msgs::GetStatesObjects srv;
    srv.request.object_id = target_object_;
    srv.request.manipulatable = true;

    if (!objects_get_client_.call(srv))
    {
      ROS_INFO_STREAM("\n\nWorld model server down\n\n");
      continue;
    }
    if (srv.response.objects.size() > 0) {
      findOject = true;
      ROS_INFO_STREAM("Find the target object");
    } else {
      if(firstTime) {
        geometry_msgs::Pose target_pose = g1::control::constructPose(
          g1::control::constructPoint(0.621, 0.461, 0.521),
          g1::control::constructQuat(0.418, 0.748, -0.216, 0.468)
        );

        std::vector<geometry_msgs::Pose> target_poses(1, target_pose);
        boost::shared_ptr<g1::control::Move> move_ptr(new g1::control::Move("User click pose"));
        move_ptr->setTargetPoses("left", target_poses);

        motion_controller_.addActionTarget(move_ptr);
        if (!motion_controller_.run()) {
          ROS_ERROR("Execution Failed. Aborting User clicking.");
          // return ;
          continue;
        }
        firstTime = false;
      }

      std::cout << "Canot find " << target_object_ << std::endl;
      setState(-10);
      ros::Duration(3.0).sleep();
    }
  }
  std::string arm_use = "left";
  motion_controller_.resetArms(arm_use);

  ROS_INFO_STREAM("Finished task of specifying object");

  setState(10);
  ros::Duration(1.5).sleep();

}

void LogicState::detectMicrowave() {

  ROS_INFO_STREAM("Detecting microwave");
  Microwave microwave(node_handle_);

  //========================
  // Read Information from world model
  //=========================
  std::vector<world_model_msgs::Object> objects = readWorldModel("", false);
  motion_controller_.updatePlanningScene(objects);

  objects = readWorldModel("", true);
  motion_controller_.updatePlanningScene(objects);

  microwave.run(&motion_controller_);
  if (microwave.updateWorldStates()) {
    ROS_INFO("Updated microwave in world states.");
  }

  std::string arm_use = "left";
  motion_controller_.resetArms(arm_use);

  ROS_INFO_STREAM("Finished task of detecting microwave");
 
  setState(40);
  ros::Duration(1.5).sleep();

}

void LogicState::openMicrowave()
{
  std::string arm_use = "left";

  motion_controller_.resetArms(arm_use);
  motion_controller_.calibrateGrippers(arm_use);
  motion_controller_.initGrippers(arm_use);
  motion_controller_.clearPlanningScene();

  //========================
  // Read Information from world model
  //=========================
  std::vector<world_model_msgs::Object> objects = readWorldModel("", false);
  motion_controller_.updatePlanningScene(objects);

  objects = readWorldModel("", true);
  motion_controller_.updatePlanningScene(objects);

  objects = readWorldModel("microwave_handle", false);
  if (objects.size() < 1) {
    ROS_ERROR_STREAM("Cound not find microwave handle");
    setState(-1);
    return;
  }

  world_model_msgs::Object microwave_handle = objects[0];

  objects = readWorldModel("microwave_door", false);
  if (objects.size() < 1) {
    ROS_ERROR_STREAM("Cound not find microwave door");
    setState(-1);
    return;
  }

  world_model_msgs::Object microwave_door = objects[0];

  // ===============================
  // Test openning microwave
  // ===============================
  // setState(2);
  
  bool success = motion_controller_.openMicrowave(microwave_handle);
  if (success) { 
    // Plan Succeed
    success = motion_controller_.run();
  } 
  
  if (!success) { 
    // Plan / Execution Failed
    ROS_ERROR("Cannot open microwave");
    setState(-2);
    return;    
  }

  // ===============================
  // Test update world model 
  // ===============================
  // setState(4);

  // Compute the opened door and handle
  geometry_msgs::Pose door_pose = microwave_door.primitive_poses[0];
  Eigen::Quaternionf door_quat(door_pose.orientation.w,
    door_pose.orientation.x,
    door_pose.orientation.y,
    door_pose.orientation.z);
  Eigen::Matrix3f rotation = door_quat.matrix();
  Eigen::Vector3f x_dir = -1.0 * rotation.col(0);
  Eigen::Vector3f y_dir = -1.0 * rotation.col(1);
  Eigen::Vector3f z_dir = rotation.col(2);
  Eigen::Vector3f offset_vector;

  offset_vector = microwave_door.primitives[0].dimensions[1] / 2.0 * y_dir 
                  - microwave_door.primitives[0].dimensions[1] / 2.0 * x_dir; 
  microwave_door.primitive_poses[0] = g1::control::updatePose(microwave_door.primitive_poses[0], 
    offset_vector(0), offset_vector(1), offset_vector(2));
  microwave_door.primitive_poses[0] = g1::control::rotatePose(microwave_door.primitive_poses[0], 0, 0, -1.57);
  
  offset_vector = microwave_door.primitives[0].dimensions[1] * y_dir 
                  - microwave_door.primitives[0].dimensions[1] * x_dir; 
  microwave_handle.primitive_poses[0] = g1::control::updatePose(microwave_handle.primitive_poses[0], 
    offset_vector(0), offset_vector(1), offset_vector(2));
  microwave_handle.primitive_poses[0] = g1::control::rotatePose(microwave_handle.primitive_poses[0], 0, 0, -1.57);

  std::vector<world_model_msgs::Object> objects_info;
  objects_info.push_back(microwave_handle);
  objects_info.push_back(microwave_door);

  world_model_msgs::UpdateStatesObjects srv;
  srv.request.objects_info = objects_info;
  srv.request.operation = srv.request.UPDATE;

  if (objects_update_client_.call(srv)) {
    ROS_INFO("Updated world model");
  }
  else {
    ROS_ERROR("Failed to call service getting from world model.");
    setState(-4);
  }

  ROS_INFO_STREAM("Finished task of openning microwave");
  setState(6);
}

void LogicState::closeMicrowave()
{
  std::string arm_use = "left";

  motion_controller_.resetArms(arm_use);
  // motion_controller.calibrateGrippers(arm_use);
  motion_controller_.initGrippers(arm_use);
  motion_controller_.clearPlanningScene();

  //========================
  // Read Information from world model
  //=========================
  std::vector<world_model_msgs::Object> objects = readWorldModel("", false);
  motion_controller_.updatePlanningScene(objects);

  objects = readWorldModel("", true);
  motion_controller_.updatePlanningScene(objects);

  objects = readWorldModel("microwave_handle", false);
  if (objects.size() < 1) {
    ROS_ERROR_STREAM("Cound not find microwave handle");
    setState(-1);
    return;
  }

  world_model_msgs::Object microwave_handle = objects[0];

  objects = readWorldModel("microwave_door", false);
  if (objects.size() < 1) {
    ROS_ERROR_STREAM("Cound not find microwave door");
    setState(-1);
    return;
  }

  world_model_msgs::Object microwave_door = objects[0];

  objects = readWorldModel("microwave_frame", false);
  if (objects.size() < 1) {
    ROS_ERROR_STREAM("Cound not find microwave frame");
    setState(-1);
    return;
  }

  world_model_msgs::Object microwave_frame = objects[0];

  // ===============================
  // Test closing microwave
  // ===============================
  // setState(2);
  
  bool success = motion_controller_.closeMicrowave(microwave_door);
  if (success) { 
    // Plan Succeed
    success = motion_controller_.run();
  } 
  
  if (!success) { 
    // Plan / Execution Failed
    ROS_ERROR("Cannot close microwave");
    setState(-24);
    return;    
  }

  // ===============================
  // Test update world model 
  // ===============================
  setState(44);


  // Compute the opened door and handle
  geometry_msgs::Pose microwave_pose = microwave_frame.primitive_poses[0];
  Eigen::Quaternionf microwave_quat(microwave_pose.orientation.w,
    microwave_pose.orientation.x,
    microwave_pose.orientation.y,
    microwave_pose.orientation.z);
  Eigen::Matrix3f rotation = microwave_quat.matrix();
  Eigen::Vector3f x_dir = -1.0 * rotation.col(0);
  Eigen::Vector3f y_dir = -1.0 * rotation.col(1);
  Eigen::Vector3f z_dir = rotation.col(2);
  Eigen::Vector3f offset_vector;

  offset_vector = - microwave_door.primitives[0].dimensions[1] / 2.0 * y_dir 
                  + microwave_door.primitives[0].dimensions[1] / 2.0 * x_dir; 
  microwave_door.primitive_poses[0] = g1::control::updatePose(microwave_door.primitive_poses[0], 
    offset_vector(0), offset_vector(1), offset_vector(2));
  microwave_door.primitive_poses[0] = g1::control::rotatePose(microwave_door.primitive_poses[0], 0, 0, 1.57);
  
  offset_vector = - microwave_door.primitives[0].dimensions[1] * y_dir 
                  + microwave_door.primitives[0].dimensions[1] * x_dir; 
  microwave_handle.primitive_poses[0] = g1::control::updatePose(microwave_handle.primitive_poses[0], 
    offset_vector(0), offset_vector(1), offset_vector(2));
  microwave_handle.primitive_poses[0] = g1::control::rotatePose(microwave_handle.primitive_poses[0], 0, 0, 1.57);

  std::vector<world_model_msgs::Object> objects_info;
  objects_info.push_back(microwave_handle);
  objects_info.push_back(microwave_door);

  world_model_msgs::UpdateStatesObjects srv;
  srv.request.objects_info = objects_info;
  srv.request.operation = srv.request.UPDATE;

  if (objects_update_client_.call(srv)) {
    ROS_INFO("Updated world model");
  }
  else {
    ROS_ERROR("Failed to call service getting from world model.");
    setState(-4);
  }

  ROS_INFO_STREAM("Finished task of closing microwave");
  ros::Duration(2).sleep();
  setState(5);
}

void LogicState::moveObject(const std::string &object_id, const world_model_msgs::Location &location) 
{

  std::string arm_use = "right";

  motion_controller_.resetArms(arm_use);
  // motion_controller.calibrateGrippers(arm_use);
  motion_controller_.initGrippers(arm_use);
  motion_controller_.clearPlanningScene();

  //========================
  // Read Information from world model
  //=========================
  std::vector<world_model_msgs::Object> objects = readWorldModel("", false);
  motion_controller_.updatePlanningScene(objects);

  objects = readWorldModel("", true);
  motion_controller_.updatePlanningScene(objects);

  objects = readWorldModel(object_id, true);
  if (objects.size() < 1) {
  	ROS_ERROR_STREAM("Cound not find object " << object_id);
  	setState(-1);
  	return;
  }

  // ===============================
  // Test pick up object 
  // ===============================
  setState(2);

  world_model_msgs::Object target_object = objects[0];

  ROS_INFO_STREAM(target_object);
  int grasp_type = -1;
  bool enable_vertical = false;

  bool success = motion_controller_.pickObject(arm_use, target_object, grasp_type, enable_vertical);
  if (success) { 
    // Pick Plan Succeed
    success = motion_controller_.run();
  } 
  else { 
    ROS_WARN("Cannot grasp object via horizontal side. Trying with vertical approachs.");
    success = motion_controller_.pickObject(arm_use, target_object, grasp_type, true);
    // Plan failed
    // TODO:: Fallback to rerun detection and planning
  }

  if (!success) { 
    // Pick Execution Failed
    ROS_ERROR("Cannot pick up object");
    setState(-2);
    return;    
  }
  
  // ===============================
  // Test place object 
  // ===============================
  setState(3);
  geometry_msgs::Point target_location = target_object.primitive_poses[0].position;
  target_location = location.position;
  
  // if (place_location != "") {
  //   world_model_msgs::QueryLocations srv;
  //   srv.request.operation = srv.request.READ;
  //   srv.request.location_id = place_location;

  //   if (location_service.call(srv)) {
  //     if (srv.response.locations.size() > 0) {
  //       target_location = srv.response.locations[0].position;

  //       double height = 0;
  //       if (target_object.primitives[0].type == target_object.primitives[0].BOX) {
  //         height = target_object.primitives[0].dimensions[2];
  //       }
  //       else if (target_object.primitives[0].type == target_object.primitives[0].CYLINDER) {
  //         height = target_object.primitives[0].dimensions[0];
  //       }

  //       target_location.z += height / 2.0;

  //       ROS_INFO_STREAM(target_location);
  //       ROS_INFO("Get location from world model");
  //     }
  //   }
  //   else {
  //     ROS_ERROR("Failed to call location service getting from world model.");
  //   }
  // }

  double height = 0;
  if (target_object.primitives[0].type == target_object.primitives[0].BOX) {
    height = target_object.primitives[0].dimensions[2];
  }
  else if (target_object.primitives[0].type == target_object.primitives[0].CYLINDER) {
    height = target_object.primitives[0].dimensions[0];
  }

  target_location.z += height / 2.0;

  success = motion_controller_.placeObject(arm_use, target_location, target_object, grasp_type);

  if (success) { 
    // Place Plan Succeed
    success = motion_controller_.run();
  } 
  else { 
    // Plan failed
    // TODO:: Fallback to rerun detection and planning

    // Plan to place to the original location
    ROS_WARN_STREAM("Failed to plan place object, put into original location");
    motion_controller_.reset();
    success = motion_controller_.placeObject(arm_use, target_object.primitive_poses[0].position, target_object, grasp_type);
    if (success) {
      motion_controller_.run();
    }
  }

  if (!success) { 
    // Place Execution Failed
    ROS_ERROR("Cannot place object");
    setState(-3);
    return;     
  }
  
  // ===============================
  // Test update world model 
  // ===============================
  setState(4);

  motion_controller_.updateObjectInfo(target_object);
  std::vector<world_model_msgs::Object> objects_info(1, target_object);
  
  world_model_msgs::UpdateStatesObjects srv;
  srv.request.objects_info = objects_info;
  srv.request.operation = srv.request.UPDATE;

  if (objects_update_client_.call(srv)) {
    ROS_INFO("Updated world model");
  }
  else {
    ROS_ERROR("Failed to call service getting from world model.");
    setState(-4);
  }


  ROS_INFO_STREAM("Finished task of moving object");

}

void LogicState::moveFromMicrowave(const std::string &object_id, const world_model_msgs::Location &location) 
{

  std::string arm_use = "right";

  motion_controller_.resetArms(arm_use);
  // motion_controller.calibrateGrippers(arm_use);
  motion_controller_.initGrippers(arm_use);
  motion_controller_.clearPlanningScene();

  //========================
  // Read Information from world model
  //=========================
  std::vector<world_model_msgs::Object> objects = readWorldModel("", false);
  motion_controller_.updatePlanningScene(objects);

  objects = readWorldModel("", true);
  motion_controller_.updatePlanningScene(objects);

  if (object_id == "") {
    ROS_ERROR_STREAM("Unknow object");
    setState(-1);
    return;
  }

  objects = readWorldModel("microwave_frame", false);
  if (objects.size() < 1) {
    ROS_ERROR_STREAM("Cound not find microwave ");
    setState(-1);
    return;
  }
  world_model_msgs::Object microwave_frame = objects[0];

  objects = readWorldModel(object_id, true);
  if (objects.size() < 1) {
    ROS_ERROR_STREAM("Cound not find object " << object_id);
    setState(-1);
    return;
  }

  // ===============================
  // Test pick up object 
  // ===============================
  setState(2);

  world_model_msgs::Object target_object = objects[0];

  int grasp_type = -1;

  bool success = motion_controller_.pickFromMicrowave(
      arm_use,
      target_object,
      microwave_frame,
      grasp_type);

  if (success) { 
    // Pick Plan Succeed
    success = motion_controller_.run();
  } 
  else { 
    ROS_WARN("Cannot grasp object inside microwave.");
    // success = motion_controller_.pickObject(arm_use, target_object, grasp_type, true);
    // Plan failed
    // TODO:: Fallback to rerun detection and planning
  }

  if (!success) { 
    // Pick Execution Failed
    ROS_ERROR("Cannot pick up object inside microwave");
    setState(-21);
    ros::Duration(2).sleep();
    return;    
  }
  

  // ===============================
  // Test place object 
  // ===============================
  setState(3);
  geometry_msgs::Point target_location = location.position;

  double height = 0;
  if (target_object.primitives[0].type == target_object.primitives[0].BOX) {
    height = target_object.primitives[0].dimensions[2];
  }
  else if (target_object.primitives[0].type == target_object.primitives[0].CYLINDER) {
    height = target_object.primitives[0].dimensions[0];
  }

  target_location.z += height / 2.0 ;

  success = motion_controller_.placeObject(arm_use, target_location, target_object, grasp_type);

  if (success) { 
    // Place Plan Succeed
    success = motion_controller_.run();
  } 
  else { 
    // Plan failed
    // TODO:: Fallback to rerun detection and planning

    // Plan to place to the original location
    ROS_ERROR_STREAM("Failed to plan place object");
  }

  if (!success) { 
    // Place Execution Failed
    ROS_ERROR("Cannot place object");
    setState(-3);
    return;     
  }
  
  // ===============================
  // Test update world model 
  // ===============================
  setState(4);

  motion_controller_.updateObjectInfo(target_object);
  target_object.primitive_poses[0].position = target_location;
  std::vector<world_model_msgs::Object> objects_info(1, target_object);
  
  world_model_msgs::UpdateStatesObjects srv;
  srv.request.objects_info = objects_info;
  srv.request.operation = srv.request.UPDATE;

  if (objects_update_client_.call(srv)) {
    ROS_INFO("Updated world model");
  }
  else {
    ROS_ERROR("Failed to call service getting from world model.");
    setState(-4);
  }
  // ===============================
  // Test update world model 
  // ===============================
  // setState(4);

  // motion_controller_.updateObjectInfo(target_object);
  // std::vector<world_model_msgs::Object> objects_info(1, target_object);
  
  // world_model_msgs::UpdateStatesObjects srv;
  // srv.request.objects_info = objects_info;
  // srv.request.operation = srv.request.UPDATE;

  // if (objects_update_client_.call(srv)) {
  //   ROS_INFO("Updated world model");
  // }
  // else {
  //   ROS_ERROR("Failed to call service getting from world model.");
  //   setState(-4);
  // }

  // setState(11);
  // ROS_INFO_STREAM("Finished task of moving object to microwave");

}

void LogicState::moveToMicrowave(const std::string &object_id) 
{

  std::string arm_use = "right";

  motion_controller_.resetArms(arm_use);
  // motion_controller.calibrateGrippers(arm_use);
  motion_controller_.initGrippers(arm_use);
  motion_controller_.clearPlanningScene();

  //========================
  // Read Information from world model
  //=========================
  std::vector<world_model_msgs::Object> objects = readWorldModel("", false);
  motion_controller_.updatePlanningScene(objects);

  objects = readWorldModel("", true);
  motion_controller_.updatePlanningScene(objects);

  if (object_id == "") {
    ROS_ERROR_STREAM("Unknow object");
    setState(-1);
    return;
  }

  objects = readWorldModel(object_id, true);
  if (objects.size() < 1) {
    ROS_ERROR_STREAM("Cound not find object " << object_id);
    setState(-1);
    return;
  }

  // ===============================
  // Test pick up object 
  // ===============================
  setState(2);

  world_model_msgs::Object target_object = objects[0];

  ROS_INFO_STREAM(target_object);
  int grasp_type = -1;
  bool enable_vertical = false;

  bool success = motion_controller_.pickObject(arm_use, target_object, grasp_type, enable_vertical);
  if (success) { 
    // Pick Plan Succeed
    success = motion_controller_.run();
  } 
  else { 
    ROS_WARN("Cannot grasp object via horizontal side.");
    // success = motion_controller_.pickObject(arm_use, target_object, grasp_type, true);
    // Plan failed
    // TODO:: Fallback to rerun detection and planning
  }

  if (!success) { 
    // Pick Execution Failed
    ROS_ERROR("Cannot pick up object");
    setState(-21);
    ros::Duration(2).sleep();
    setState(11);
    return;    
  }
  
  // ===============================
  // Test place object 
  // ===============================
  setState(3);

  objects = readWorldModel("microwave_frame", false);
  if (objects.size() < 1) {
    ROS_ERROR_STREAM("Cound not find microwave ");
    ROS_WARN_STREAM("Failed to plan place object, put into original location");
    motion_controller_.reset();
    success = motion_controller_.placeObject(arm_use, target_object.primitive_poses[0].position, target_object, grasp_type);
    if (success) {
      motion_controller_.run();
    }

    setState(-1);
    return;
  }
  
  world_model_msgs::Object microwave_frame = objects[0];

  success = motion_controller_.placeIntoMicrowave(arm_use, target_object, microwave_frame, grasp_type);

  if (success) { 
    // Place Plan Succeed
    success = motion_controller_.run();
  } 
  else { 
    // Plan failed
    // TODO:: Fallback to rerun detection and planning

    // Plan to place to the original location
    ROS_WARN_STREAM("Failed to plan place object, put into original location");
    setState(7);
    motion_controller_.reset();
    success = motion_controller_.placeObject(arm_use, target_object.primitive_poses[0].position, target_object, grasp_type);
    if (success) {
      motion_controller_.run();
    }
  }

  if (!success) { 
    // Place Execution Failed
    ROS_ERROR("Cannot place object");
    setState(-22);
    return;     
  }
  
  // ===============================
  // Test update world model 
  // ===============================
  // setState(4);

  motion_controller_.updateObjectInfo(target_object);
  std::vector<world_model_msgs::Object> objects_info(1, target_object);
  
  world_model_msgs::UpdateStatesObjects srv;
  srv.request.objects_info = objects_info;
  srv.request.operation = srv.request.UPDATE;

  if (objects_update_client_.call(srv)) {
    ROS_INFO("Updated world model");
  }
  else {
    ROS_ERROR("Failed to call service getting from world model.");
    setState(-4);
  }

  setState(11);
  ROS_INFO_STREAM("Finished task of moving object to microwave");

}

std::vector<world_model_msgs::Object> LogicState::readWorldModel(const std::string &object_id, bool manipulatable)
{

  world_model_msgs::GetStatesObjects srv;
  srv.request.object_id = object_id;
  srv.request.manipulatable = manipulatable;

  std::vector<world_model_msgs::Object> objects;
  if (objects_get_client_.call(srv)) {
    objects = srv.response.objects;
  }
  else {
    ROS_ERROR("Failed to call service getting from world model.");
  }
  return objects;
}


}

}


