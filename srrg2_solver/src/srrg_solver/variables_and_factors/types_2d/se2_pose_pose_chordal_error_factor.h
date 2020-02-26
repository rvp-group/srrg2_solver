#pragma once
#include <srrg_geometry/geometry2d.h>

#include "srrg_solver/solver_core/error_factor.h"
#include "variable_se2.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  class SE2PosePoseChordalErrorFactor : public ErrorFactor_<6, VariableSE2Right, VariableSE2Right>,
                                        public MeasurementOwnerEigen_<Isometry2f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void errorAndJacobian(bool error_only_ = false) final;

  protected:
    inline Vector6f flattenIsometry(const Isometry2f& T) {
      Vector6f returned;
      returned.setZero();
      returned.block<2, 1>(0, 0) = T.linear().block<2, 1>(0, 0);
      returned.block<2, 1>(2, 0) = T.linear().block<2, 1>(0, 1);
      returned.block<2, 1>(4, 0) = T.translation();
      return returned;
    }
  };
} // namespace srrg2_solver
