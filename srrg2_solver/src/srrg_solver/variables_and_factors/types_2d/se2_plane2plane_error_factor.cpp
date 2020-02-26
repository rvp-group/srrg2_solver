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

  SE2Plane2PlaneWithSensorErrorFactorAD::ADErrorVectorType SE2Plane2PlaneWithSensorErrorFactorAD::
  operator()(VariableTupleType& vars) {
    _is_valid = true;

    const auto& X           = vars.at<0>()->adEstimate();
    ADEstimateType X_sensor = _ad_robot_in_sensor * X;
    const auto& R           = X_sensor.linear();

    Vector2_<DualValuef> moving = Vector2_<DualValuef>::Zero();
    convertMatrix(moving, *_moving_point);
    const Vector2_<DualValuef> p_pred = X_sensor * moving;

    moving.setZero();
    convertMatrix(moving, *_moving_normal);
    const Vector2_<DualValuef> n_pred = R * moving;

    Vector2_<DualValuef> fixed_point, fixed_normal;
    convertMatrix(fixed_point, *_fixed_point);
    convertMatrix(fixed_normal, *_fixed_normal);

    Eigen::Matrix<DualValuef, 1, 2> nt = Eigen::Matrix<DualValuef, 1, 2>::Zero();
    convertMatrix(nt, _fixed_normal->transpose());

    ADErrorVectorType e = ADErrorVectorType::Zero();

    e(0)        = nt * (p_pred - fixed_point);
    e.tail<2>() = n_pred - fixed_normal;

    return e;
  }

  INSTANTIATE(SE2Plane2PlaneWithSensorErrorFactorAD)
  INSTANTIATE(SE2Plane2PlaneWithSensorErrorFactorADCorrespondenceDriven)

} // namespace srrg2_solver
