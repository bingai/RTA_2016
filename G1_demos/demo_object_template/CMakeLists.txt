cmake_minimum_required(VERSION 2.8.3)
project(demo_object_template)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  world_model_msgs
  shape_msgs
  geometry_msgs
  g1_control
)

catkin_package()

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(demo_object_template src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(demo_object_template
  ${catkin_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS demo_object_template
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

