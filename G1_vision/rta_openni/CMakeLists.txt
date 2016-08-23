cmake_minimum_required(VERSION 2.8.3)
project(rta_openni)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

#========================== Check for C++11 compliance & set flags=============
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else ()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()


#=========================== Find catkin macros and libraries =================
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        pcl_ros
        std_msgs
        message_generation
        message_filters
        image_transport
        cv_bridge
        tf
        camera_info_manager
        )

#========================== Find other dependencies ===========================
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d highgui)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package (OpenNI2 REQUIRED)
find_package(X11 QUIET)

#find_package(PCL 1.8.0 EXACT REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#add_message_files( FILES ColorFrame.msg DepthFrame.msg PointCloud.msg PointCloudTS.msg)

## Generate services in the 'srv' folder
add_service_files( FILES
        GetCameraIntrinsics.srv
        GetCameraExtrinsics.srv
        SetIrStream.srv
        SetEmitterState.srv
        UseHighResolutionColor.srv
        GetCurrentFrames.srv
        )

## Generate actions in the 'action' folder
# add_action_files(STX
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES rta_openni
        CATKIN_DEPENDS message_runtime roscpp rospy std_msgs pcl_ros image_transport cv_bridge
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
#include_directories(
#        ${catkin_INCLUDE_DIRS}
#)

## Declare a C++ library
# add_library(rta_openni
#   src/${PROJECT_NAME}/rta_openni.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(rta_openni ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


file(GLOB_RECURSE COMMON_SOURCES src/common/*.cpp)
#========================== COLLECT SOURCES FOR & ADD MAIN NODE EXECUTABLE =============================================
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES src/node/*.cpp ${COMMON_SOURCES})
set(MAIN_TARGET ${PROJECT_NAME}_node)
add_executable(${MAIN_TARGET} ${${PROJECT_NAME}_SOURCES})

target_include_directories(${MAIN_TARGET} PRIVATE ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${OPENNI2_INCLUDE_DIRS})
target_include_directories(${MAIN_TARGET} PUBLIC include)
target_link_libraries(${MAIN_TARGET} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OPENNI2_LIBRARIES})

add_dependencies(${MAIN_TARGET} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${MAIN_TARGET} ${PROJECT_NAME}_gencpp)

#========================== COLLECT SOURCES FOR & ADD POINTS NODE EXECUTABLE ===========================================
set(POINTS_TARGET points)
file(GLOB_RECURSE ${POINTS_TARGET}_SOURCES src/points/*.cpp ${COMMON_SOURCES})
add_executable(${POINTS_TARGET} ${${POINTS_TARGET}_SOURCES})

target_include_directories(${POINTS_TARGET} PRIVATE
        ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS} ${OPENNI2_INCLUDE_DIRS})
target_include_directories(${POINTS_TARGET} PUBLIC include)
target_link_libraries(${POINTS_TARGET} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${OPENNI2_LIBRARIES})

add_dependencies(${POINTS_TARGET} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${POINTS_TARGET} ${PROJECT_NAME}_gencpp)

#========================== COLLECT SOURCES FOR & ADD CALIBRATION_CAPTURE EXECUTABLE ===================================
set(CALIB_CAPTURE_TARGET calibration_capture)
file(GLOB_RECURSE ${CALIB_CAPTURE_TARGET}_SOURCES src/calibration_capture/*.cpp)
add_executable(${CALIB_CAPTURE_TARGET} ${${CALIB_CAPTURE_TARGET}_SOURCES})

target_include_directories(${CALIB_CAPTURE_TARGET} PRIVATE ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
        ${X11_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
target_include_directories(${CALIB_CAPTURE_TARGET} PUBLIC include)
target_link_libraries(${CALIB_CAPTURE_TARGET} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${X11_LIBRARIES} ${Boost_LIBRARIES})

if(X11_FOUND)
    target_compile_definitions(${CALIB_CAPTURE_TARGET} PRIVATE X11_FOUND=1)
endif()

add_dependencies(${CALIB_CAPTURE_TARGET} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${CALIB_CAPTURE_TARGET} ${PROJECT_NAME}_gencpp)

#========================== TEST SUBSCRIBER NODE SOURCES & TARGET ======================================================
set(TEST_SUBSCRIBER rta_openni_test_node)
set(${TEST_SUBSCRIBER}_SOURCES src/test_node/rta_openni_test_node.cpp)
add_executable(${TEST_SUBSCRIBER} ${${TEST_SUBSCRIBER}_SOURCES})

target_include_directories(${TEST_SUBSCRIBER} PRIVATE ${catkin_INCLUDE_DIRS})
target_include_directories(${TEST_SUBSCRIBER} PUBLIC include)
target_link_libraries(${TEST_SUBSCRIBER} ${catkin_LIBRARIES})

add_dependencies(${TEST_SUBSCRIBER} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${TEST_SUBSCRIBER} ${PROJECT_NAME}_gencpp)

#========================= TRANSFORM READER EXECUTABLE =================================================================

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(ra_openni_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(ra_openni_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS rta_openni ra_openni_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ra_openni.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
