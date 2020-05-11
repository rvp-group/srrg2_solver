#include "se2_plane2plane_error_factor.h"

//! include this: this contains all the implementations of the factors
//! that are hidden to the modules that do not need them to avoid excessive compilation times (EVIL)
#include "srrg_solver/solver_core/ad_error_factor_impl.cpp"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  void SE2Plane2PlaneErrorFactor::errorAndJacobian(bool error_only) {
    _is_valid                           = true;
    const auto& X                       = this->_variables.template at<0>()->estimate();
    const Matrix2f& R                   = X.linear();
    const Vector2f p_pred               = X * (*_moving_point);
    const Vector2f n_pred               = R * (*_moving_normal);
    const Eigen::Matrix<float, 1, 2> nt = _fixed_normal->transpose();
    _e.setZero();
    _e(0)        = nt * (p_pred - (*_fixed_point));
    _e.tail<2>() = n_pred - (*_fixed_normal);
    if (error_only) {
      return;
    }
    _J.setZero();
    _J.block<1, 2>(0, 0) = nt * R;
    _J(0, 2)             = -nt * R * srrg2_core::geometry2d::skew(*_moving_point);
    _J.block<2, 1>(1, 2) = -R * srrg2_core::geometry2d::skew(*_moving_normal);
  }

  INSTANTIATE(SE2Plane2PlaneErrorFactor)
  INSTANTIATE(SE2Plane2PlaneErrorFactorCorrespondenceDriven)

  void SE2Plane2PlaneWithSensorErrorFactor::errorAndJacobian(bool error_only) {
    _is_valid             = true;
    const Isometry2f X    = _robot_in_sensor * _variables.template at<0>()->estimate();
    const Matrix2f& R     = X.linear();
    const Vector2f p_pred = X * (*_moving_point);
    const Vector2f n_pred = R * (*_moving_normal);
    const Eigen::Matrix<float, 1, 2> nt = _fixed_normal->transpose();
    _e.setZero();
    _e(0)        = nt * (p_pred - (*_fixed_point));
    _e.tail<2>() = n_pred - (*_fixed_normal);
    if (error_only) {
      return;
    }
    _J.setZero();
    _J.block<1, 2>(0, 0) = nt * R;
    _J(0, 2)             = -nt * R * srrg2_core::geometry2d::skew(*_moving_point);
    _J.block<2, 1>(1, 2) = -R * srrg2_core::geometry2d::skew(*_moving_normal);
  }

  INSTANTIATE(SE2Plane2PlaneWithSensorErrorFactor)
  INSTANTIATE(SE2Plane2PlaneWithSensorErrorFactorCorrespondenceDriven)

} // namespace srrg2_solver
