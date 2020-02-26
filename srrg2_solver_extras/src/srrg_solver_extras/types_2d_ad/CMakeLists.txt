add_library(srrg2_solver_extras_types2d_library SHARED
            instances.cpp instances.h
            se2_point2point_left_error_factor_ad.cpp se2_point2point_left_error_factor_ad.h
            se2_pose_point_bearing_left_error_factor_ad.cpp se2_pose_point_bearing_left_error_factor_ad.h
            se2_pose_point_left_error_factor_ad.cpp se2_pose_point_left_error_factor_ad.h
            se2_pose_pose_left_error_factor_ad.cpp se2_pose_pose_left_error_factor_ad.h
            )

target_link_libraries(srrg2_solver_extras_types2d_library srrg2_solver_core_library srrg2_solver_types2d_library)
