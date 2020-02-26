#include "se3_prior_euler_error_factor_ad.h"
#include <srrg_solver/solver_core/ad_error_factor_impl.cpp>
#include <srrg_solver/solver_core/error_factor_impl.cpp>
#include <srrg_solver/solver_core/instance_macros.h>

namespace srrg2_solver_extras {
  INSTANTIATE(SE3PriorEulerErrorFactorAD)
}
