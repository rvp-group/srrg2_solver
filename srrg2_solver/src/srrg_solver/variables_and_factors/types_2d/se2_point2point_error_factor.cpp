#include "se2_point2point_error_factor.h"

//! include this: this contains all the implementations of the factors
//! that are hidden to the modules that do not need them to avoid excessive compilation times (EVIL)
#include "srrg_solver/solver_core/ad_error_factor_impl.cpp"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  void SE2Point2PointErrorFactor::errorAndJacobian(bool error_only) {
    const EstimateType& X = this->_variables.template at<0>()->estimate();
    const Matrix2f& R     = X.linear();
    _e                    = X * (*moving) - (*fixed);
    if (error_only) {
      return;
    }
    _J.block<2, 2>(0, 0) = R;
    _J.block<2, 1>(0, 2) = -R * geometry2d::skew(*moving);
  }

  INSTANTIATE(SE2Point2PointErrorFactor)
  INSTANTIATE(SE2Point2PointErrorFactorCorrespondenceDriven)

  void SE2Point2PointWithSensorErrorFactor::errorAndJacobian(bool error_only) {
    const EstimateType& X = _robot_in_sensor * this->_variables.template at<0>()->estimate();
    const Matrix2f& R     = X.linear();
    _e                    = X * (*moving) - (*fixed);
    if (error_only) {
      return;
    }
    _J.block<2, 2>(0, 0) = R;
    _J.block<2, 1>(0, 2) = -R * geometry2d::skew(*moving);
  }

  INSTANTIATE(SE2Point2PointWithSensorErrorFactor)
  INSTANTIATE(SE2Point2PointWithSensorErrorFactorCorrespondenceDriven)

} // namespace srrg2_solver
