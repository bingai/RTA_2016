cmake_minimum_required(VERSION 2.8.3)
project(reconstruction_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
  shape_msgs
  geometry_msgs
)

add_message_files(
  FILES
  RegisteredView.msg
  Reconstruction.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  shape_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
