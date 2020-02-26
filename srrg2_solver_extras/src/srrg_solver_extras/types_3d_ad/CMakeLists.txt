add_library(srrg2_solver_extras_types3d_library SHARED
            instances.cpp instances.h
            se3_point2point_euler_error_factor_ad.cpp se3_point2point_euler_error_factor_ad.h
            se3_pose_point_euler_error_factor_ad.cpp se3_pose_point_euler_error_factor_ad.h
            se3_pose_point_offset_euler_error_factor_ad.cpp se3_pose_point_offset_euler_error_factor_ad.h
            se3_pose_pose_chordal_quaternion_error_factor_ad.cpp se3_pose_pose_chordal_quaternion_error_factor_ad.h
            se3_pose_pose_error_factor_ad.cpp se3_pose_pose_error_factor_ad.h
            se3_pose_pose_euler_error_factor_ad.cpp se3_pose_pose_euler_error_factor_ad.h
            se3_prior_euler_error_factor_ad.cpp se3_prior_euler_error_factor_ad.h
            )

target_link_libraries(srrg2_solver_extras_types3d_library srrg2_solver_core_library srrg2_solver_types3d_library)
