add_library(srrg2_solver_factor_graph_utils_library SHARED
  #view selector
  factor_graph_view_selector.cpp factor_graph_view_selector.h
  #visit
  factor_graph_visit_entry.cpp factor_graph_visit_entry.h
  factor_graph_visit_cost.cpp factor_graph_visit_cost.h
  factor_graph_visit_policy.cpp factor_graph_visit_policy.h
  factor_graph_visit.cpp factor_graph_visit.h
  # initializer
  factor_graph_initializer_rule.cpp factor_graph_initializer_rule.h
  factor_graph_initializer.cpp factor_graph_initializer.h
  # closure validator
  factor_graph_closure_validator.cpp factor_graph_closure_validator.h
    
  # instances
  instances.cpp instances.h
  )

target_link_libraries(srrg2_solver_factor_graph_utils_library
  srrg2_solver_types2d_library
  srrg2_solver_types3d_library
  srrg2_solver_calib_library
  srrg2_solver_core_library
  )
