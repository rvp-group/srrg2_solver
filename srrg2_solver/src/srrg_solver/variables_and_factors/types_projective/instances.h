#pragma once
#include "se3_projective_depth_error_factor.h"
#include "se3_projective_error_factor.h"
#include "se3_rectified_stereo_projective_error_factor.h"

// ldg similiarity variables
#include "variable_sim3.h"
#include "variable_sim3_ad.h"
// // ldg similiarity factors
#include "sim3_point2point_error_factor.h"
#include "sim3_pose_pose_error_factor_ad.h"
#include "se3_pose_point_omni_ba_error_factor.h"

namespace srrg2_solver {
  void variables_and_factors_projective_registerTypes() __attribute__((constructor));
}
