#pragma once
#include "srrg_solver/solver_core/error_factor.h"
#include "srrg_solver/solver_core/measurement_owner.h"
#include "variable_point3.h"
#include "variable_se3.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  class SE3PosePointErrorFactor
    : public ErrorFactor_<3, VariableSE3QuaternionRight, VariablePoint3>,
      public MeasurementOwnerEigen_<Vector3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void errorAndJacobian(bool error_only_ = false) final;
  };
} // namespace srrg2_solver
