# #ia multisolver app, you give a graph, it optimizes it
add_executable(solver_app_graph_optimizer solver_app_graph_optimizer.cpp)
target_link_libraries(solver_app_graph_optimizer
  srrg2_solver_types2d_library
  srrg2_solver_types3d_library
  srrg2_solver_calib_library
  ${SOLVER_EXAMPLES_CORE_LIBRARIES}
  ${CHOLMOD_LIBRARIES}
  ${CSPARSE_LIBRARY}
  ${catkin_LIBRARIES}
)

#ia converter app. given a g2o file it becames a boss graph and viceversa
add_executable(solver_app_graph_converter solver_app_graph_converter.cpp)
target_link_libraries(solver_app_graph_converter
  srrg2_g2o_converter_library
  ${SOLVER_EXAMPLES_CORE_LIBRARIES}
  ${catkin_LIBRARIES}
)
