cmake_minimum_required(VERSION 2.8.3)
project(interactive_marker_ui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  world_model_msgs
  message_generation
)

add_service_files(
	FILES
	location_update_status.srv
	location_update_confirm.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES interactive_marker_ui
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
	scripts/interactive.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)