#pragma once
#include "srrg_solver/solver_core/variable.h"
#include <srrg_geometry/geometry2d.h>

namespace srrg2_solver {

  using namespace srrg2_core;

  // point variable
  struct VariablePoint2 : public Variable_<2, Vector2_> {
    using BaseVariableType = VariablePoint2;

    virtual void setZero();
    virtual void applyPerturbation(const Vector2f& pert);
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace srrg2_solver
