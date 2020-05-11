#include "se3_prior_offset_error_factor_ad.h"
#include "srrg_solver/solver_core/ad_error_factor_impl.cpp"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {

  SE3PriorOffsetErrorFactorAD::ADErrorVectorType SE3PriorOffsetErrorFactorAD::
  operator()(VariableTupleType& vars_) {
    Isometry3_<DualValuef> prediction_ad =
      _robot_in_sensor_ad * vars_.at<0>()->adEstimate() * _sensor_in_robot_ad;
    return geometry3d::t2v(_inv_measure_ad * prediction_ad);
  }

  INSTANTIATE(SE3PriorOffsetErrorFactorAD)
} // namespace srrg2_solver
