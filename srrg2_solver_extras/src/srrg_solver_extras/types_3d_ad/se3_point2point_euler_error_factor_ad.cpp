#include "se3_point2point_euler_error_factor_ad.h"
#include <srrg_solver/solver_core/ad_error_factor_impl.cpp>
#include <srrg_solver/solver_core/error_factor_impl.cpp>
#include <srrg_solver/solver_core/instance_macros.h>

namespace srrg2_solver_extras {
  INSTANTIATE(SE3Point2PointEulerErrorFactorAD)
  INSTANTIATE(SE3Point2PointEulerErrorFactorCorrespondenceDrivenAD)
} // namespace srrg2_solver_extras
