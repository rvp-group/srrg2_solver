####################################
#         SOLVER EXAMPLES          #
####################################

#ds readability
set(SOLVER_EXAMPLES_CORE_LIBRARIES
    srrg2_solver_core_library
    srrg2_boss_library
    srrg2_property_library
)

# # multi solver (hot topic)
add_executable(example_sparse_solver_block_inverse example_sparse_solver_block_inverse.cpp)
target_link_libraries(example_sparse_solver_block_inverse
  srrg2_solver_types2d_library
  srrg2_solver_types3d_library
  srrg2_solver_factor_graph_utils_library
  srrg2_g2o_converter_library
  ${SOLVER_EXAMPLES_CORE_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_executable(example_levemberg example_levemberg.cpp)
target_link_libraries(example_levemberg
  srrg2_solver_types2d_library
  srrg2_solver_factor_graph_utils_library
  ${SOLVER_EXAMPLES_CORE_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(example_linear_solver_block_inverse example_linear_solver_block_inverse.cpp)
target_link_libraries(example_linear_solver_block_inverse
  ${SOLVER_EXAMPLES_CORE_LIBRARIES}
  ${catkin_LIBRARIES})
  
add_executable(example_linear_block_solver_cholmod_full example_linear_block_solver_cholmod_full.cpp)
target_link_libraries(example_linear_block_solver_cholmod_full
  ${SOLVER_EXAMPLES_CORE_LIBRARIES}
  ${catkin_LIBRARIES})

# # visit (hot topic)
add_executable(example_factor_graph_visit example_factor_graph_visit.cpp)
target_link_libraries(example_factor_graph_visit
  srrg2_solver_types2d_library
  srrg2_solver_types3d_library
  srrg2_solver_calib_library
  srrg2_solver_factor_graph_utils_library
  ${SOLVER_EXAMPLES_CORE_LIBRARIES}
  ${CHOLMOD_LIBRARIES}
  ${CSPARSE_LIBRARY}
  ${catkin_LIBRARIES}
)

# # initializer (hot topic)
add_executable(example_factor_graph_initializer example_factor_graph_initializer.cpp)
target_link_libraries(example_factor_graph_initializer
  srrg2_solver_types2d_library
  srrg2_solver_types3d_library
  srrg2_solver_calib_library
  srrg2_solver_factor_graph_utils_library
  ${SOLVER_EXAMPLES_CORE_LIBRARIES}
  ${CHOLMOD_LIBRARIES}
  ${CSPARSE_LIBRARY}
  ${catkin_LIBRARIES}
)

# tg example ICP 2D
add_executable(example_se2_icp example_se2_icp.cpp)
target_link_libraries(example_se2_icp
  srrg2_solver_core_library
  srrg2_solver_types2d_library
  ${catkin_LIBRARIES}
)


#ia example PGO 3D
add_executable(example_se3_pgo example_se3_pgo.cpp)
target_link_libraries(example_se3_pgo
  srrg2_solver_core_library
  srrg2_solver_types3d_library
  ${catkin_LIBRARIES}
)
