#include "se2_pose_point_bearing_error_factor.h"


//! include this: this contains all the implementations of the factors
//! that are hidden to the modules that do not need them to avoid excessive compilation times (EVIL)
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  void SE2PosePointBearingErrorFactor::errorAndJacobian(bool error_only_) {
    const VariableSE2Right* pose = _variables.at<0>();
    const VariablePoint2* point  = _variables.at<1>();
    const float& angle           = this->_measurement(0);

    const Isometry2f& transformation = pose->estimate();
    const Vector2f& landmark         = point->estimate();
    const Matrix2f& R                = transformation.linear();

    const Vector2f predicted_point = transformation.inverse() * landmark;
    const float predicted_angle    = std::atan2(predicted_point(1), predicted_point(0));

    // tg equivalent to trace(I - R_z^T * R_prediction) [box-minus]
    _e(0) = 2.f * (1 - cos(predicted_angle - angle));
    if (error_only_) {
      return;
    }

    // tg derivative of the trace with respect to the predicted angle
    float dtrace_dprediction = 2.f * sin(predicted_angle - angle);
    // tg Jacobian of the bearing prediction with respect to the predicted point
    Vector2f J_atan = -1.0f * geometry2d::skew(predicted_point);
    J_atan /= predicted_point.squaredNorm();
    auto J_pose              = jacobian<0>();
    J_pose.block<1, 2>(0, 0) = -dtrace_dprediction * J_atan.transpose();
    J_pose(0, 2) = dtrace_dprediction * J_atan.transpose() * geometry2d::skew(predicted_point);
    auto J_point = jacobian<1>();
    J_point      = dtrace_dprediction * J_atan.transpose() * R.transpose();
  }

  INSTANTIATE(SE2PosePointBearingErrorFactor)

} // namespace srrg2_solver
