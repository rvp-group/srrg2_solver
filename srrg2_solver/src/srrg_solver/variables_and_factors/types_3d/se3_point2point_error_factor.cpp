#include "se3_point2point_error_factor.h"
#include "srrg_solver/solver_core/ad_error_factor_impl.cpp"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  void SE3Point2PointErrorFactor::errorAndJacobian(bool error_only) {
    const Isometry3f& X    = _variables.template at<0>()->estimate();
    const Vector3f& moving = *(this->moving);
    const Vector3f& fixed  = *(this->fixed);
    const Matrix3f& R      = X.linear();
    _e                     = X * moving - fixed;
    if (error_only) {
      return;
    }
    _J.block<3, 3>(0, 0) = R;
    _J.block<3, 3>(0, 3) = -2.f * R * geometry3d::skew(moving);
  }

  INSTANTIATE(SE3Point2PointErrorFactor)
  INSTANTIATE(SE3Point2PointErrorFactorCorrespondenceDriven)

  void SE3Point2PointWithSensorErrorFactor::errorAndJacobian(bool error_only) {
    const Isometry3f& X    = _robot_in_sensor * _variables.template at<0>()->estimate();
    const Vector3f& moving = *(this->moving);
    const Vector3f& fixed  = *(this->fixed);
    const Matrix3f& R      = X.linear();
    _e                     = X * moving - fixed;
    if (error_only) {
      return;
    }
    _J.block<3, 3>(0, 0) = R;
    _J.block<3, 3>(0, 3) = -2.f * R * geometry3d::skew(moving);
  }

  INSTANTIATE(SE3Point2PointWithSensorErrorFactor)
  INSTANTIATE(SE3Point2PointWithSensorErrorFactorCorrespondenceDriven)

} // namespace srrg2_solver
