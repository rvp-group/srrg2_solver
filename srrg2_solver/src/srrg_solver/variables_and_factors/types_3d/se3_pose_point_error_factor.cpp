#include "se3_pose_point_error_factor.h"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  void SE3PosePointErrorFactor::errorAndJacobian(bool error_only_) {
    const Isometry3f& X       = _variables.at<0>()->estimate();
    const Vector3f& p        = _variables.at<1>()->estimate();
    Vector3f predicted_point = X.inverse() * p;
    _e                       = predicted_point - _measurement;
    if (error_only_) {
      return;
    }
    _J.setZero();
    // tg jacobian with respect to pose
    _J.block<3, 3>(0, 0) = -1.f * Matrix3f::Identity();
    _J.block<3, 3>(0, 3) = 2.f * geometry3d::skew(predicted_point);
    // tg jacobian with respect to point
    _J.block<3, 3>(0, 6) = X.linear().transpose();

  }
  INSTANTIATE(SE3PosePointErrorFactor)
} // namespace srrg2_solver
