cmake_minimum_required(VERSION 2.8.3)
project(object_cls_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
  object_cls_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp
)


