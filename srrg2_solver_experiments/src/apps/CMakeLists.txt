catkin_add_gtest(gtest_bench_eth_tilting_laser gtest_bench_eth_tilting_laser.cpp)
target_link_libraries(gtest_bench_eth_tilting_laser
  srrg2_point_cloud_library
  srrg2_system_utils_library
  srrg2_solver_core_library
  srrg2_solver_extras_types2d_library
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

catkin_add_gtest(gtest_bench_icl gtest_bench_icl.cpp)
target_link_libraries(gtest_bench_icl
  srrg2_point_cloud_library
  srrg2_system_utils_library
  srrg2_solver_core_library
  srrg2_solver_extras_types2d_library
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

catkin_add_gtest(gtest_bench_stanford_bunny gtest_bench_stanford_bunny.cpp)
target_link_libraries(gtest_bench_stanford_bunny
  srrg2_point_cloud_library
  srrg2_system_utils_library
  srrg2_solver_core_library
  srrg2_solver_extras_types2d_library
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
