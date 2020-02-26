add_library(srrg2_g2o_converter_library SHARED
  # tg converter
  g2o_converter_action_base.h
  g2o_converter_action_se2.h
  g2o_converter_action_se2.cpp
  g2o_converter_action_se3.h
  g2o_converter_action_se3.cpp
  g2o_converter_actions.h
  g2o_converter.h
  g2o_converter.cpp
)

target_link_libraries(srrg2_g2o_converter_library 
  srrg2_solver_types2d_library 
  srrg2_solver_types3d_library
  srrg2_solver_projective_library
  srrg2_solver_calib_library
  srrg2_solver_core_library
  )