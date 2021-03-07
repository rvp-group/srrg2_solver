add_library(srrg2_solver_calib_utils_library SHARED
  #graph assembler
  calib_graph_2d_assembler.cpp calib_graph_2d_assembler.h
  calib_graph_3d_assembler.cpp calib_graph_3d_assembler.h
  # instances
  instances.cpp instances.h
  )

target_link_libraries(srrg2_solver_calib_utils_library
  srrg2_solver_calib_library
  srrg2_solver_types2d_library
  srrg2_solver_types3d_library
  srrg2_solver_core_library
  srrg2_messages_library
  )
