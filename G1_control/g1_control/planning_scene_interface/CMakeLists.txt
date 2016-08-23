set(G1_LIB_NAME g1_control_planning_scene_interface)

add_library(${G1_LIB_NAME} src/planning_scene_interface.cpp)
target_link_libraries(${G1_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${G1_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/ DESTINATION include)
