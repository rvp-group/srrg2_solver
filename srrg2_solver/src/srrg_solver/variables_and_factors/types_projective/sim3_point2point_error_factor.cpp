#include "sim3_point2point_error_factor.h"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  void Sim3Point2PointErrorFactor::errorAndJacobian(bool error_only) {
    const Similiarity3f& X = _variables.template at<0>()->estimate();
    const Vector3f& moving = *(this->moving);
    const Vector3f& fixed  = *(this->fixed);
    const Matrix3f& R      = X.linear();
    const float s          = 1.f / X.inverseScaling();
    _e                     = X * moving - fixed;
    if (error_only) {
      return;
    }

    _J.block<3, 3>(0, 0) = s * R;
    _J.block<3, 3>(0, 3) = -2.f * s * R * geometry3d::skew(moving);
    _J.block<3, 1>(0, 6) = -s * R * moving;
  }

  INSTANTIATE(Sim3Point2PointErrorFactor)
  INSTANTIATE(Sim3Point2PointErrorFactorCorrespondenceDriven)
} // namespace srrg2_solver
