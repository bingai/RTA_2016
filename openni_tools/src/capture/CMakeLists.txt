cmake_minimum_required(VERSION 2.8)

project(openni_capture)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../bin/")

### ----------------------------------------------------------------------------
### OpenNI dependent options
### ----------------------------------------------------------------------------

find_package (OpenNI2 REQUIRED)
message (STATUS "")
if (OPENNI2_FOUND)
    
  message (STATUS " --------------------------")
  message (STATUS " Found OpenNI2")
  message (STATUS "     * include dirs: ${OPENNI2_INCLUDE_DIRS}")
  message (STATUS "     * libraries:    ${OPENNI2_LIBRARIES}")
else()
  message (FATAL_ERROR : "OpenNI2 NOT FOUND!")
endif()

find_package(PNG REQUIRED)

### ----------------------------------------------------------------------------
### Add libraries
### ----------------------------------------------------------------------------
if (${OpenCV_VERSION} VERSION_LESS 3.0.0)
  add_library(${CAPTURE_LIB_TARGET} SHARED ./openni_capture.cpp ./bitstrm.cpp ./grfmt_base.cpp ./grfmt_png.cpp ./loadsave.cpp)
else()
  add_library(${CAPTURE_LIB_TARGET} SHARED ./openni_capture.cpp)
endif()

target_link_libraries (${CAPTURE_LIB_TARGET} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${OPENNI2_LIBRARIES} ${PNG_LIBRARIES} )
target_include_directories(${CAPTURE_LIB_TARGET} PRIVATE ${OPENNI2_INCLUDE_DIRS} ${PNG_INCLUDE_DIRS})
target_compile_definitions(${CAPTURE_LIB_TARGET} PRIVATE ${OPENNI2_DEFINITIONS} ${PNG_DEFINITIONS})


### ----------------------------------------------------------------------------
### Add executables
### ----------------------------------------------------------------------------

# Frame capture
SET(CAPTURE_EXE_TARGET openni_capture_util)
add_executable( ${CAPTURE_EXE_TARGET} ../../applications/capture/openni_capture_main.cpp)
target_link_libraries (${CAPTURE_EXE_TARGET} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${CAPTURE_LIB_TARGET})


# View frames
SET(VIEWER_EXE_TARGET openni_viewer)
add_executable(${VIEWER_EXE_TARGET} ../../applications/capture/openni_viewer.cpp)
target_link_libraries (${VIEWER_EXE_TARGET} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${CAPTURE_LIB_TARGET})

#-------------------------- INSTALLATION --------------------------------------
install(TARGETS
        ${CAPTURE_LIB_TARGET}
        ${CAPTURE_EXE_TARGET}
        ${VIEWER_EXE_TARGET}
        ${INSTALL_TARGETS_DEFAULT_ARGS}
        )