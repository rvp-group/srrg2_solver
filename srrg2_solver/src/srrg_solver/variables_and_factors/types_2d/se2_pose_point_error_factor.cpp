#include "se2_pose_point_error_factor.h"

//! include this: this contains all the implementations of the factors
//! that are hidden to the modules that do not need them to avoid excessive compilation times (EVIL)
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {

  using namespace srrg2_core;

  void SE2PosePointErrorFactor::errorAndJacobian(bool error_only) {
    const Isometry2f& X = _variables.at<0>()->estimate();
    const Vector2f& p   = _variables.at<1>()->estimate();
    Vector2f prediction = X.inverse() * p;
    _e                  = prediction - _measurement;
    if (error_only) {
      return;
    }
    const Matrix2f& R = X.linear().transpose();
    auto J_pose       = jacobian<0>();
    auto J_point      = jacobian<1>();

    J_pose.block<2, 2>(0, 0) = -1.0f * Matrix2f::Identity();
    J_pose.block<2, 1>(0, 2) = geometry2d::skew(prediction);
    J_point                  = R;
    // std::cerr << "Total Jacobian : \n" << totalJacobian() << std::endl;
  }

  INSTANTIATE(SE2PosePointErrorFactor)

} // namespace srrg2_solver
