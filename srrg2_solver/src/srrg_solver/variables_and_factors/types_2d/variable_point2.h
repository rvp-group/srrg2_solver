#pragma once
#include "srrg_solver/solver_core/variable.h"
#include <srrg_geometry/geometry2d.h>

namespace srrg2_solver {

  using namespace srrg2_core;

  /** @brief 2D Point Variable.
   */
  class VariablePoint2 : public Variable_<2, Vector2_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseVariableType = VariablePoint2;

    virtual ~VariablePoint2() = default;
    virtual void setZero() override;
    virtual void applyPerturbation(const Vector2f& pert) override;
  };

} // namespace srrg2_solver
