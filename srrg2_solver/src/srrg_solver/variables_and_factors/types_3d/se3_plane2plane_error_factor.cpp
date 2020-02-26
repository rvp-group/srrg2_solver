#include "se3_plane2plane_error_factor.h"
#include "srrg_solver/solver_core/factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  void SE3Plane2PlaneErrorFactor::errorAndJacobian(bool error_only) {
    const Isometry3f& X = _variables.template at<0>()->estimate();
    
    const Matrix3f& R = X.linear();

    const Vector3f& p_moving            = *moving_point;
    const Vector3f& n_moving            = *moving_normal;
    const Vector3f& p_fixed             = *fixed_point;
    const Vector3f& n_fixed             = *fixed_normal;
    const Vector3f p_pred               = X * p_moving;
    const Vector3f n_pred               = R * n_moving;
    const Eigen::Matrix<float, 1, 3> nt = n_fixed.transpose();

    _e.setZero();
    _e(0)                = nt * (p_pred - p_fixed);
    _e.block<3, 1>(1, 0) = n_pred - n_fixed;
    if (error_only)
      return;

    _J.setZero();
    _J.block<1, 3>(0, 0) = nt * R;
    _J.block<1, 3>(0, 3) = -2.f * nt * R * srrg2_core::geometry3d::skew(p_moving);
    _J.block<3, 3>(1, 3) = -2.f * R * srrg2_core::geometry3d::skew(n_moving);
    this->_is_valid      = true;
  }

  INSTANTIATE(SE3Plane2PlaneErrorFactorCorrespondenceDriven)
} // namespace srrg2_solver
