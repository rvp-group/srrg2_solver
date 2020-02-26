add_library(srrg2_solver_projective_library SHARED
            # tg camera based factors
            se3_projective_error_factor.cpp se3_projective_error_factor.h
            se3_rectified_stereo_projective_error_factor.cpp se3_rectified_stereo_projective_error_factor.h
            se3_projective_depth_error_factor.cpp se3_projective_depth_error_factor.h
            instances.cpp instances.h
  )
  
target_link_libraries(srrg2_solver_projective_library srrg2_solver_types3d_library srrg2_solver_core_library)
