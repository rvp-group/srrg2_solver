#include "se3_pose_motion_error_factor_ad.h"
#include "srrg_solver/solver_core/ad_error_factor_impl.cpp"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  INSTANTIATE(SE3PoseMotionErrorFactorAD)
  INSTANTIATE(SE3PoseMotionErrorFactorDataDriven)
}
