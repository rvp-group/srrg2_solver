#pragma once
#include "srrg_solver/solver_core/error_factor.h"
#include "srrg_solver/solver_core/measurement_owner.h"
#include "variable_point3.h"
#include "variable_se3.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  /** @brief 3D Pose-point factor. Estimates the robot pose and the landmark position.
   * Error is computed by the difference of landmark's prediction in robot's coordinates and the
   * measurement gathered
   */
  class SE3PosePointErrorFactor
    : public ErrorFactor_<3, VariableSE3QuaternionRight, VariablePoint3>,
      public MeasurementOwnerEigen_<Vector3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void errorAndJacobian(bool error_only_ = false) final;
    void _drawImpl(ViewerCanvasPtr canvas_) const override;
  };
} // namespace srrg2_solver
