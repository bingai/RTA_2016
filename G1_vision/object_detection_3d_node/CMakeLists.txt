cmake_minimum_required(VERSION 2.8.3)
project(object_detection_3d_node)

# First compile the table_segmentation library
include_directories (../object_detection_3d/include)
add_subdirectory (../object_detection_3d ${CMAKE_CURRENT_BINARY_DIR}/object_detection_3d)

#############################
## Add system dependencies ##
#############################

## Catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  world_model_msgs
  object_cls_msgs
  reconstruction_msgs
  shape_msgs  
  message_generation
  rviz_visual_tools
  eigen_conversions
  cv_bridge
)

## PCL
find_package(PCL 1.8.0 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files ( DIRECTORY srv
                    FILES
                    ObjectDetection3dService.srv
                  )

## Generate added messages and services with any dependencies listed here
generate_messages ( DEPENDENCIES
                    std_msgs
                    sensor_msgs
                    world_model_msgs
                    object_cls_msgs
                    reconstruction_msgs
                  )

###################################
## catkin specific configuration ##
###################################

catkin_package(
 CATKIN_DEPENDS message_runtime
)

### ----------------------------------------------------------------------------
### C++11 support
### ----------------------------------------------------------------------------

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories (include )
include_directories (${catkin_INCLUDE_DIRS} )

# Node executable
add_executable (object_detection_3d_node src/object_detection_3d_node.cpp)
add_dependencies (object_detection_3d_node  object_detection_3d_node_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries (object_detection_3d_node ${catkin_LIBRARIES} object_detection_3d)

# add_executable (object_detection_3d_node_demo src/object_detection_3d_node_demo.cpp)
# add_dependencies (object_detection_3d_node_demo  object_detection_3d_node_generate_messages_cpp # ${catkin_EXPORTED_TARGETS})
# target_link_libraries (object_detection_3d_node_demo ${catkin_LIBRARIES} object_detection_3d)

# ## Declare a C++ executable
add_executable (object_detection_3d_node_test src/object_detection_3d_node_test.cpp)
add_dependencies (object_detection_3d_node_test  object_detection_3d_node_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries (object_detection_3d_node_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})
