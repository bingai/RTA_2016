cmake_minimum_required(VERSION 2.8.3)
project(object_detection_2d)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  message_generation
  genmsg
  cv_bridge
  image_transport
  tf
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_detection_2d
  CATKIN_DEPENDS roscpp visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(object_detection_2d
#   src/${PROJECT_NAME}/object_detection_2d.cpp
# )

## Declare a C++ executable
add_executable(table_marker src/table_marker.cpp)
add_executable(top_down_calib src/top_down_calib.cpp)


## Add cmake target dependencies of the executable
add_dependencies(table_marker ${catkin_EXPORTED_TARGETS})
add_dependencies(top_down_calib ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
target_link_libraries(table_marker ${catkin_LIBRARIES})
target_link_libraries(top_down_calib ${catkin_LIBRARIES})


