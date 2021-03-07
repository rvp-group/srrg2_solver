add_library(srrg2_solver_projective_library SHARED
            # tg camera based factors
            se3_projective_error_factor.cpp se3_projective_error_factor.h
            se3_rectified_stereo_projective_error_factor.cpp se3_rectified_stereo_projective_error_factor.h
            se3_projective_depth_error_factor.cpp se3_projective_depth_error_factor.h

             # ldg similiarity factors
            variable_sim3.cpp
            variable_sim3_ad.cpp variable_sim3_ad.h 
            sim3_point2point_error_factor.cpp sim3_point2point_error_factor.h
            sim3_pose_pose_error_factor_ad.cpp sim3_pose_pose_error_factor_ad.h
            se3_pose_point_omni_ba_error_factor.cpp se3_pose_point_omni_ba_error_factor.h
            instances.cpp instances.h
  )
  
target_link_libraries(srrg2_solver_projective_library srrg2_solver_types3d_library srrg2_solver_core_library)
