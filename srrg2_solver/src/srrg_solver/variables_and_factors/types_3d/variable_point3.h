#pragma once
#include "srrg_solver/solver_core/variable.h"
#include <srrg_geometry/geometry3d.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  /** @brief 3D Point Variable.
   */
  class VariablePoint3 : public Variable_<3, Vector3_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BaseVariableType = VariablePoint3;

    virtual ~VariablePoint3() = default;
    virtual void setZero() override;

    virtual void applyPerturbation(const Vector3f& pert) override;

    void _drawImpl(ViewerCanvasPtr canvas_) const override;
  };

} // namespace srrg2_solver
