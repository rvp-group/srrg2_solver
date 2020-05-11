#tg block inversion
catkin_add_gtest(test_linear_solver_block_inverse test_linear_solver_block_inverse.cpp)
target_link_libraries(test_linear_solver_block_inverse
    srrg2_solver_core_library
    srrg2_boss_library
    srrg2_property_library
    ${catkin_LIBRARIES}
)
  
#ia 2D icp test
catkin_add_gtest(test_se2_icp test_se2_icp.cpp)
target_link_libraries(test_se2_icp
  srrg2_solver_core_library
  srrg2_solver_types2d_library
  ${catkin_LIBRARIES}
)

# #bdc 2D n(ormal)-icp test
catkin_add_gtest(test_se2_nicp test_se2_nicp.cpp)
target_link_libraries(test_se2_nicp
  srrg2_solver_core_library
  srrg2_solver_types2d_library
  ${catkin_LIBRARIES}
)
# tg prior factors SE2
catkin_add_gtest(test_se2_prior_factor test_se2_prior_factor.cpp)
target_link_libraries(test_se2_prior_factor
  srrg2_solver_core_library
  srrg2_solver_types2d_library
  ${catkin_LIBRARIES}
)

#tg prior factors SE3
catkin_add_gtest(test_se3_prior_factor test_se3_prior_factor.cpp)
target_link_libraries(test_se3_prior_factor
  srrg2_solver_core_library
  srrg2_solver_types3d_library
  ${catkin_LIBRARIES}
)

# 3D n(ormal)-icp test
catkin_add_gtest(test_se3_nicp test_se3_nicp.cpp)
target_link_libraries(test_se3_nicp
  srrg2_solver_core_library
  srrg2_solver_types3d_library
  ${catkin_LIBRARIES}
)

# #ia 3D icp test
catkin_add_gtest(test_se3_icp test_se3_icp.cpp)
target_link_libraries(test_se3_icp
  srrg2_solver_core_library
  srrg2_solver_types3d_library
  ${catkin_LIBRARIES}
)

#bdc 3D motion-based-calibration test
catkin_add_gtest(test_se3_motion_based_calib test_se3_motion_based_calib.cpp)
target_link_libraries(test_se3_motion_based_calib
  srrg2_solver_core_library
  srrg2_solver_types3d_library
  ${catkin_LIBRARIES}
)

# #tg pose point offset test
catkin_add_gtest(test_se3_multi_point_registration test_se3_multi_point_registration.cpp)
target_link_libraries(test_se3_multi_point_registration
  srrg2_solver_core_library
  srrg2_solver_types3d_library
  ${catkin_LIBRARIES}
)

# tg 3D pgo test
catkin_add_gtest(test_se3_pgo test_se3_pgo.cpp)
target_link_libraries(test_se3_pgo
  srrg2_solver_core_library
  srrg2_solver_types3d_library
  ${catkin_LIBRARIES}
)

# #tg test cholmod linear solver on se3 pgo problems
catkin_add_gtest(test_sparse_block_linear_solver_cholmod_full test_sparse_block_linear_solver_cholmod_full.cpp)
target_link_libraries(test_sparse_block_linear_solver_cholmod_full
  srrg2_solver_core_library
  srrg2_solver_types3d_library
  ${catkin_LIBRARIES}
)

# #tg se2 pgo test using pose pose chordal error factor (no AD)
catkin_add_gtest(test_se2_pgo test_se2_pgo.cpp)
target_link_libraries(test_se2_pgo
   srrg2_solver_core_library
   srrg2_solver_types2d_library
   ${catkin_LIBRARIES})

# #tg se2 pose point factor test 
catkin_add_gtest(test_se2_multi_point_registration test_se2_multi_point_registration.cpp)
target_link_libraries(test_se2_multi_point_registration
   srrg2_solver_core_library
   srrg2_solver_types2d_library
   ${catkin_LIBRARIES})
   
# #ia 3D pgo test
catkin_add_gtest(test_se3_matchables test_se3_matchables.cpp)
target_link_libraries(test_se3_matchables
  srrg2_solver_core_library
  srrg2_solver_types3d_library
  ${catkin_LIBRARIES}
)

#ds g2o converter test
catkin_add_gtest(test_g2o_graph_converter test_g2o_graph_converter.cpp)
target_link_libraries(test_g2o_graph_converter
  srrg2_solver_core_library
  srrg2_solver_types3d_library
  srrg2_g2o_converter_library
  ${SOLVER_EXAMPLES_CORE_LIBRARIES}
  ${catkin_LIBRARIES}
)

#ia moved here
add_executable(factor_graph_loop_validator_test
  factor_graph_loop_validator_test.cpp)
target_link_libraries(factor_graph_loop_validator_test
  srrg2_solver_factor_graph_utils_library
  srrg2_solver_types3d_library
  srrg2_solver_types2d_library
  srrg2_solver_core_library
  srrg2_boss_library
  srrg2_property_library
  ${CHOLMOD_LIBRARIES}
  ${CSPARSE_LIBRARY}
  ${catkin_LIBRARIES}
)

# tg dominik factors in new format
catkin_add_gtest(test_se3_posit test_se3_posit.cpp)
target_link_libraries(test_se3_posit
  srrg2_solver_projective_library
  ${catkin_LIBRARIES}
)


 # ldg test pose pose offset with sim3 variable
 catkin_add_gtest(test_se3_pose_pose_offset_error_factor test_se3_pose_pose_offset_error_factor.cpp)
 target_link_libraries(test_se3_pose_pose_offset_error_factor
   srrg2_solver_core_library
   srrg2_solver_projective_library
   ${catkin_LIBRARIES}
 )


# ldg test similiarity variable
catkin_add_gtest(test_sim3 test_sim3.cpp)
target_link_libraries(test_sim3
  srrg2_solver_core_library
  srrg2_solver_projective_library
  ${catkin_LIBRARIES}
)

# ldg test pgo with sim3 variable
catkin_add_gtest(test_sim3_pgo test_sim3_pgo.cpp)
 target_link_libraries(test_sim3_pgo
   srrg2_solver_core_library
   srrg2_solver_projective_library
   ${catkin_LIBRARIES}
)


# ldg test icp with sim3 variable
# catkin_add_gtest(test_sim3_icp test_sim3_icp.cpp)
# target_link_libraries(test_sim3_icp
#   srrg2_solver_core_library
#   srrg2_solver_projective_library
#   ${catkin_LIBRARIES}
# )
