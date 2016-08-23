cmake_minimum_required(VERSION 2.8.3)
project(hand_tracker_2d)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  image_transport
  cv_bridge
)


################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
  HandBBox.msg
)

# add_service_files(
#   FILES
# )
  
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS /usr/local/include
#  LIBRARIES ${KCFTRACKER}
  CATKIN_DEPENDS message_runtime
#  DEPENDS kcftracker
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(hand_tracker_2d src/hand_tracker_2d.cpp)

## Add cmake target dependencies of the executable
add_dependencies(hand_tracker_2d ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
target_link_libraries(hand_tracker_2d 
	${catkin_LIBRARIES} 
	/usr/local/lib/libkcftracker.so
)




