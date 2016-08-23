cmake_minimum_required(VERSION 2.8.3)
project(workspace_detection_node)

find_package (catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  world_model_msgs
  shape_msgs  
  message_generation
  eigen_conversions
  # rviz_visual_tools
  # cv_bridge
)

find_package(PCL 1.8.0 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

add_service_files ( DIRECTORY srv
                    FILES
                    WorkspaceDetection.srv
                  )

generate_messages ( DEPENDENCIES
                    std_msgs
                    sensor_msgs
                    world_model_msgs
                  )

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include (CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

include_directories (include)
include_directories (${catkin_INCLUDE_DIRS})

set(SISYPHUS_LIBS sisyphus_graph_utilities sisyphus_registration_utilities sisyphus_sift_engine sisyphus_box_detection sisyphus_box_fitting sisyphus_pointcloud_utilities sisyphus_io_utilities sisyphus_sift_rgbd_slam)

# Utilities library
add_library (workspace_detection_utilities src/workspace_detection_utilities.cpp)
target_link_libraries (workspace_detection_utilities ${SISYPHUS_LIBS} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Node executable
add_executable (workspace_detection_node src/workspace_detection_node.cpp)
add_dependencies (workspace_detection_node workspace_detection_utilities workspace_detection_node_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries (workspace_detection_node workspace_detection_utilities ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Tester executable
add_executable (workspace_detection_node_test src/workspace_detection_node_test.cpp)
add_dependencies (workspace_detection_node_test workspace_detection_node_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries (workspace_detection_node_test ${catkin_LIBRARIES})
