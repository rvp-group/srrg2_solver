#include "se2_point2point_error_factor.h"

//! include this: this contains all the implementations of the factors
//! that are hidden to the modules that do not need them to avoid excessive compilation times (EVIL)
#include "srrg_solver/solver_core/ad_error_factor_impl.cpp"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  void SE2Point2PointErrorFactor::errorAndJacobian(bool error_only) {
    const auto& X     = this->_variables.template at<0>()->estimate();
    const Matrix2f& R = X.linear();
    _e                = X * (*moving) - (*fixed);
    if (error_only) {
      return;
    }
    _J.block<2, 2>(0, 0) = R;
    _J.block<2, 1>(0, 2) = -R * geometry2d::skew(*moving);
  }

  INSTANTIATE(SE2Point2PointErrorFactor)
  INSTANTIATE(SE2Point2PointErrorFactorCorrespondenceDriven)

  SE2Point2PointWithSensorErrorFactorAD::ADErrorVectorType SE2Point2PointWithSensorErrorFactorAD::
  operator()(VariableTupleType& vars) {
    const auto& X = vars.at<0>()->adEstimate();
    Vector2_<DualValuef> moving, fixed;
    convertMatrix(moving, *_moving);
    convertMatrix(fixed, *_fixed);
    ADErrorVectorType e = (_ad_sensor_in_robot_inverse * X) * moving - fixed;
    return e;
  }

  INSTANTIATE(SE2Point2PointWithSensorErrorFactorAD)
  INSTANTIATE(SE2Point2PointWithSensorErrorFactorADCorrespondenceDriven)

} // namespace srrg2_solver
