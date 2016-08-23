cmake_minimum_required(VERSION 2.8.3)
project(dummy_vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  world_model_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dummy_vision
  CATKIN_DEPENDS roscpp rospy std_msgs world_model_msgs
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(dummy_vision src/dummy_vision.cpp)
add_dependencies(dummy_vision dummy_vision_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(dummy_vision ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS dummy_vision
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
