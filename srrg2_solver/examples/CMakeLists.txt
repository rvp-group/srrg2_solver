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
add_executable(example_se2_bearing_only_slam example_se2_bearing_only_slam.cpp)
 target_link_libraries(example_se2_bearing_only_slam srrg2_solver_types2d_library
  srrg2_solver_factor_graph_utils_library 
  ${SOLVER_EXAMPLES_CORE_LIBRARIES} 
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
