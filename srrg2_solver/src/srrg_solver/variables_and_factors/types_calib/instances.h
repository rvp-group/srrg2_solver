#pragma once
#include "differential_drive_odom_error_factor_ad.h"
#include "differential_drive_odom_predictor_ad.h"
#include "differential_drive_odom_sensor2d_error_factor_ad.h"
#include "differential_drive_odom_sensor3d_error_factor_ad.h"
#include "differential_drive_odom_time_delay_sensor2d_error_factor_ad.h"
#include "sensor2d_extrinsic_pose_motion_calib_ad.h"
#include "sensor3d_extrinsic_pose_motion_calib_ad.h"
#include "sensor3d_pose_time_delay_error_factor_ad.h"

namespace srrg2_solver {
  void variables_and_factors_calib_registerTypes() __attribute__((constructor));
}
