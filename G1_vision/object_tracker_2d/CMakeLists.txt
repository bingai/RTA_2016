cmake_minimum_required(VERSION 2.8.3)
project(object_tracker_2d)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
  object_cls_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS /usr/local/include
#  LIBRARIES ${KCFTRACKER}
#  CATKIN_DEPENDS roscpp
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
add_executable(object_tracker_2d src/object_tracker_2d.cpp)

## Add cmake target dependencies of the executable
add_dependencies(object_tracker_2d ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
target_link_libraries(object_tracker_2d 
	${catkin_LIBRARIES} 
	/usr/local/lib/libkcftracker.so
)




