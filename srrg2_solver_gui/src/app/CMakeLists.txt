add_executable(srrg_solver_gui_app solver_action_draw.cpp srrg_solver_gui_app.cpp)
target_link_libraries(srrg_solver_gui_app
  ${CHOLMOD_LIBRARIES}
  ${CSPARSE_LIBRARY}
  ${QGLViewer_LIBRARIES}
  ${SRRG_QT_LIBRARIES}
  ${catkin_LIBRARIES}
  pthread)