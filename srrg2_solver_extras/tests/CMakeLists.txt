#tg test icp 2D
catkin_add_gtest(test_se2_icp_left_ad test_se2_icp_left_ad.cpp)
target_link_libraries(test_se2_icp_left_ad
  srrg2_solver_core_library
  srrg2_solver_extras_types2d_library
  ${catkin_LIBRARIES}
)
#tg test pose-point / pose-point-bearing
catkin_add_gtest(test_se2_multi_point_registration_left_ad test_se2_multi_point_registration_left_ad.cpp)
target_link_libraries(test_se2_multi_point_registration_left_ad
  srrg2_solver_core_library
  srrg2_solver_extras_types2d_library
  ${catkin_LIBRARIES}
)
#tg test PGO SE2
catkin_add_gtest(test_se2_pgo_left_ad test_se2_pgo_left_ad.cpp)
target_link_libraries(test_se2_pgo_left_ad
  srrg2_solver_core_library
  srrg2_solver_extras_types2d_library
  ${catkin_LIBRARIES}
)

#tg test icp 3D
catkin_add_gtest(test_se3_icp_euler_left_ad test_se3_icp_euler_left_ad.cpp)
target_link_libraries(test_se3_icp_euler_left_ad
  srrg2_solver_core_library
  srrg2_solver_extras_types3d_library
  ${catkin_LIBRARIES}
)

#tg test icp 3D
catkin_add_gtest(test_se3_multi_point_registration_euler_ad test_se3_multi_point_registration_euler_ad.cpp)
target_link_libraries(test_se3_multi_point_registration_euler_ad
  srrg2_solver_core_library
  srrg2_solver_extras_types3d_library
  ${catkin_LIBRARIES}
)

#tg test PGO SE3
catkin_add_gtest(test_se3_pgo_ad test_se3_pgo_ad.cpp)
target_link_libraries(test_se3_pgo_ad
  srrg2_solver_core_library
  srrg2_solver_extras_types3d_library
  ${catkin_LIBRARIES}
)

#tg test prior euler AD
catkin_add_gtest(test_se3_prior_euler_ad test_se3_prior_euler_ad.cpp)
target_link_libraries(test_se3_prior_euler_ad
  srrg2_solver_core_library
  srrg2_solver_extras_types3d_library
  ${catkin_LIBRARIES}
)
