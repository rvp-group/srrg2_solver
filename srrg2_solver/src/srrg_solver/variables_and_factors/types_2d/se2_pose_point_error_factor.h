#pragma once
#include "srrg_solver/solver_core/error_factor.h"
#include "variable_point2.h"
#include "variable_se2.h"
#include <srrg_geometry/geometry2d.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  /** @brief 2D Pose-point factor. Estimates the robot pose and the 2D landmark position.
   * Error is computed by the difference of landmark's prediction in robot's coordinates and the
   * measurement gathered
   */
  class SE2PosePointErrorFactor : public ErrorFactor_<2, VariableSE2Right, VariablePoint2>,
                                  public MeasurementOwnerEigen_<Vector2f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void errorAndJacobian(bool error_only_ = false) final;
  };
} // namespace srrg2_solver
