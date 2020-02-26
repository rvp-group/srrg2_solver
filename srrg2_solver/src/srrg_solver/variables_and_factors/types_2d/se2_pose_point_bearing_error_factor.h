#pragma once
#include <srrg_geometry/geometry2d.h>

#include "srrg_solver/solver_core/error_factor.h"
#include "variable_point2.h"
#include "variable_se2.h"

namespace srrg2_solver {

  class SE2PosePointBearingErrorFactor : public ErrorFactor_<1, VariableSE2Right, VariablePoint2>,
                                         public MeasurementOwnerEigen_<Vector1f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void errorAndJacobian(bool error_only_ = false) final;
  };

} // namespace srrg2_solver
