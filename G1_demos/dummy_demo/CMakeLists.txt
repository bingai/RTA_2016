cmake_minimum_required(VERSION 2.8.3)
project(dummy_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dummy_vision
)

catkin_package()

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(dummy_demo src/dummy_demo.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(dummy_demo
  ${catkin_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS dummy_demo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

