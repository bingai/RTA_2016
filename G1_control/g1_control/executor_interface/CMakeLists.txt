set(G1_CONTROL_LIB_NAME g1_control_executor_interface)

add_library(${G1_CONTROL_LIB_NAME} 
  src/executor.cpp 
  src/action.cpp 
  src/utilities.cpp
)

target_link_libraries(${G1_CONTROL_LIB_NAME} g1_control_planning_scene_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${G1_CONTROL_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/ DESTINATION include)

add_executable(control_demo src/demo.cpp)
target_link_libraries(control_demo ${G1_CONTROL_LIB_NAME} g1_control_planning_scene_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
