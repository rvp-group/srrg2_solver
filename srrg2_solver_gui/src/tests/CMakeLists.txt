add_executable(test_graph_viewer test_graph_viewer.cpp)
target_link_libraries(test_graph_viewer
  ${CHOLMOD_LIBRARIES}
  ${CSPARSE_LIBRARY}
  ${QGLViewer_LIBRARIES}
  ${SRRG_QT_LIBRARIES}
  ${catkin_LIBRARIES}
  pthread)

  
add_executable(test_pgo test_pgo.cpp)
target_link_libraries(test_pgo
  ${CHOLMOD_LIBRARIES}
  ${CSPARSE_LIBRARY}
  ${QGLViewer_LIBRARIES}
  ${SRRG_QT_LIBRARIES}
  ${catkin_LIBRARIES}
  pthread)