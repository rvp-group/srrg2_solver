add_library(srrg2_solver_calib_library SHARED
  variable_time.cpp variable_time.h
  variable_time_ad.cpp variable_time_ad.h

  differential_drive_odom_predictor_ad.cpp
  differential_drive_odom_predictor_ad.h

  differential_drive_odom_time_delay_sensor2d_error_factor_ad.cpp
  differential_drive_odom_time_delay_sensor2d_error_factor_ad.h 
  
  differential_drive_odom_error_factor_ad.cpp
  differential_drive_odom_error_factor_ad.h

  differential_drive_odom_sensor2d_error_factor_ad.cpp
  differential_drive_odom_sensor2d_error_factor_ad.h

  differential_drive_odom_sensor3d_error_factor_ad.cpp
  differential_drive_odom_sensor3d_error_factor_ad.h

  sensor3d_pose_time_delay_error_factor_ad.cpp
  sensor3d_pose_time_delay_error_factor_ad.h

  sensor2d_extrinsic_pose_motion_calib_ad.cpp sensor2d_extrinsic_pose_motion_calib_ad.h
  sensor3d_extrinsic_pose_motion_calib_ad.cpp sensor3d_extrinsic_pose_motion_calib_ad.h

  instances.h instances.cpp
  )

target_link_libraries(srrg2_solver_calib_library
  srrg2_solver_core_library
  )
