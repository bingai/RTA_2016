CMAKE_MINIMUM_REQUIRED(VERSION 2.8.8)
project(openni_tools)
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_CAPS)

# version (e.g. for packaging)
set(${PROJECT_NAME_CAPS}_MAJOR_VERSION 0)
set(${PROJECT_NAME_CAPS}_MINOR_VERSION 1)
set(${PROJECT_NAME_CAPS}_PATCH_VERSION 2)
set(${PROJECT_NAME_CAPS}_VERSION
        ${${PROJECT_NAME_CAPS}_MAJOR_VERSION}.${${PROJECT_NAME_CAPS}_MINOR_VERSION}.${${PROJECT_NAME_CAPS}_PATCH_VERSION})

# -------------- BUILD OPTIONS ------------------------------------------------------

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeModules/")

add_definitions("-Wall")

###-----------------------------------------------------------------------------
### Add dependencies
###-----------------------------------------------------------------------------

# -------------------------------------
# Cotire
# -------------------------------------

include(cotire)
set(COTIRE_MINIMUM_NUMBER_OF_TARGET_SOURCES 1)

# -------------------------------------
# CPP tools
# -------------------------------------

find_package(namaris REQUIRED)
include_directories(${Namaris_INCLUDE_DIRS})


# -------------------------------------
# OpenCV dependent options
# -------------------------------------

find_package(OpenCV REQUIRED core highgui calib3d)
message(STATUS "")
if (OpenCV_FOUND)

    message(STATUS " --------------------------")
    message(STATUS " Found OpenCV version ${OpenCV_VERSION}")
    message(STATUS "     * include dirs: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "     * library dirs: ${OpenCV_INSTALL_PATH}")
    message(STATUS "     * libraries:    ${OpenCV_LIBS}")
else ()
    message(FATAL_ERROR : "OpenCV NOT FOUND!")
endif ()
message(STATUS "")

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_INSTALL_PATH})
link_libraries(${OpenCV_LIBS})

# -------------------------------------
# PCL dependent options
# -------------------------------------

find_package(PCL REQUIRED)
message(STATUS "")
if (PCL_FOUND)
    message(STATUS "")
    message(STATUS " -----------------------")
    message(STATUS " Found PCL version ${PCL_VERSION}")
    message(STATUS "     * include dirs: ${PCL_INCLUDE_DIRS}")
    message(STATUS "     * library dirs: ${PCL_LIBRARY_DIRS}")
    message(STATUS "     * libraries:    ${PCL_LIBRARIES}")
else ()
    message(FATAL_ERROR : "PCL NOT FOUND!")
endif ()
message(STATUS "")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# -------------------------------------
# BOOST
# -------------------------------------

find_package(Boost REQUIRED COMPONENTS regex system filesystem)

if (Boost_FOUND)
    message(STATUS " --------------------------")
    message(STATUS " Found Boost version ${Boost_VERSION}")
    message(STATUS "     * include dirs: ${Boost_INCLUDE_DIRS}")
    message(STATUS "     * libraries:    ${Boost_LIBRARIES}")
endif ()

### ----------------------------------------------------------------------------
### C++11
### ----------------------------------------------------------------------------

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

### ----------------------------------------------------------------------------
### Other library depdendencies
### ----------------------------------------------------------------------------

set(INCLUDE_DIRS ./include)
include_directories(${INCLUDE_DIRS})

# --------- target names --------

SET(CAPTURE_LIB_TARGET openni_capture)
SET(CALIBRATION_LIB_TARGET openni_calibration)

# ============================ INSTALLATION ====================================
set(INSTALL_TARGETS_DEFAULT_ARGS
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

# subdirectory adding has to be has to be done here, since install command uses INSTALL_TARGETS_DEFAULT_ARGS
add_subdirectory(src/capture)
add_subdirectory(src/calibration)

# install header files
install(
        DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING
        PATTERN "*.hpp"
        PATTERN "*.h")

# uninstall target (this does not delete the directories created during installation)
configure_file(
        "${PROJECT_SOURCE_DIR}/CMakeModules/CMakeUninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)

add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE ${PROJECT_NAME})

include(CMakePackageConfigHelpers)

# Create a octomap-config.cmake file for the use from the build tree
set(${PROJECT_NAME_CAPS}_INCLUDE_DIRS "${INCLUDE_DIRS}")
set(${PROJECT_NAME_CAPS}_LIB_DIR "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
# Set library names as absolute paths
# Windows, spec. MSVC requires the .lib suffix for imported libs
if (WIN32)
    set(OPENNI_CAPTURE_LIBRARY
            "${CMAKE_IMPORT_LIBRARY_PREFIX}${CAPTURE_LIB_TARGET}${CMAKE_IMPORT_LIBRARY_SUFFIX}"
            )
    set(OPENNI_CALIBRATION_LIBRARY
            "${CMAKE_IMPORT_LIBRARY_PREFIX}${CALIBRATION_LIB_TARGET}${CMAKE_IMPORT_LIBRARY_SUFFIX}"
            )
else ()
    set(OPENNI_CAPTURE_LIBRARY
            "${CMAKE_SHARED_LIBRARY_PREFIX}${CAPTURE_LIB_TARGET}${CMAKE_SHARED_LIBRARY_SUFFIX}"
            )
    set(OPENNI_CALIBRATION_LIBRARY
            "${CMAKE_SHARED_LIBRARY_PREFIX}${CALIBRATION_LIB_TARGET}${CMAKE_SHARED_LIBRARY_SUFFIX}"
            )
endif ()


CONFIGURE_PACKAGE_CONFIG_FILE(
        ${PROJECT_NAME}-config.cmake.in
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
        PATH_VARS ${PROJECT_NAME_CAPS}_INCLUDE_DIRS ${PROJECT_NAME_CAPS}_LIB_DIR
        INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME})

WRITE_BASIC_PACKAGE_VERSION_FILE(
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
        VERSION ${${PROJECT_NAME_CAPS}_VERSION}
        COMPATIBILITY AnyNewerVersion)

# Create a openni_tools-config.cmake file for the use from the install tree
# and install it
set(${PROJECT_NAME_CAPS}_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
set(${PROJECT_NAME_CAPS}_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib")

CONFIGURE_PACKAGE_CONFIG_FILE(
        ${PROJECT_NAME}-config.cmake.in
        "${PROJECT_BINARY_DIR}/InstallFiles/${PROJECT_NAME}-config.cmake"
        PATH_VARS ${PROJECT_NAME_CAPS}_INCLUDE_DIRS ${PROJECT_NAME_CAPS}_LIB_DIR
        INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME})

WRITE_BASIC_PACKAGE_VERSION_FILE(
        ${PROJECT_BINARY_DIR}/InstallFiles/${PROJECT_NAME}-config-version.cmake
        VERSION ${${PROJECT_NAME_CAPS}_VERSION}
        COMPATIBILITY AnyNewerVersion)

install(FILES
        "${PROJECT_BINARY_DIR}/InstallFiles/${PROJECT_NAME}-config.cmake"
        "${PROJECT_BINARY_DIR}/InstallFiles/${PROJECT_NAME}-config-version.cmake"
        DESTINATION share/${PROJECT_NAME}/)

# Write pkgconfig-file:
include(InstallPkgConfigFile)
install_pkg_config_file(${PROJECT_NAME}
        CFLAGS
        LIBS -l${CAPTURE_LIB_TARGET} -l${CALIBRATION_LIB_TARGET}
        REQUIRES
        VERSION ${${PROJECT_NAME_CAPS}_VERSION})


