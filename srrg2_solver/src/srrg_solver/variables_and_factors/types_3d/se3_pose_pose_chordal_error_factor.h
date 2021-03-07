#pragma once
#include "srrg_solver/solver_core/error_factor.h"
#include "variable_se3.h"

namespace srrg2_solver {
  /**
   * @brief 3D PGO error factor exploiting chordal distance.
   * Error vector is 12 component (from h(x) - Z)
   * vertices use EULER angles with increment PRE-multiplied
   */
  class SE3PosePoseChordalEulerLeftErrorFactor
    : public ErrorFactor_<12, VariableSE3EulerLeft, VariableSE3EulerLeft>,
      public MeasurementOwnerEigen_<Isometry3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = ErrorFactor_<12, VariableSE3EulerLeft, VariableSE3EulerLeft>;
    using Scalar   = MeasurementType::Scalar;
    void errorAndJacobian(bool error_only_ = false) final;

    void _drawImpl(ViewerCanvasPtr canvas_) const override;
  };
} // namespace srrg2_solver
