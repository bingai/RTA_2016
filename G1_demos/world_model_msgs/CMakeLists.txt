cmake_minimum_required(VERSION 2.8.3)
project(world_model_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
  shape_msgs
  geometry_msgs
  reconstruction_msgs
)

add_message_files(
  FILES
  Object.msg
  Location.msg
)

add_service_files(
  FILES
  UpdateStatesDummy.srv
  GetStatesDummy.srv
  PointCloud2Service.srv
  GetStatesObjects.srv
  UpdateStatesObjects.srv
  ImageCoordinate.srv
  exec_state.srv
  QueryLocations.srv
  cmd.srv
  ProgramOrder.srv
  ReconstructionUpdate.srv
  ReconstructionQuery.srv
  ReconstructedPointCloudQuery.srv
)
  
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  shape_msgs
  geometry_msgs
  reconstruction_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
