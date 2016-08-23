cmake_minimum_required(VERSION 2.8.3)
project(object_cls_msgs)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
)

add_message_files(
  FILES
  BBox.msg
  BBoxList.msg
)

add_service_files(
  FILES
  GetObjectClass.srv
)
  
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)


