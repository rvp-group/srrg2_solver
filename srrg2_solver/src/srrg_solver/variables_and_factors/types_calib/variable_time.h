#pragma once
#include <srrg_geometry/geometry_defs.h>
#include <srrg_solver/solver_core/variable.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  /**
   * @brief 1D Variable Time
   */
  class VariableTime : public Variable_<1, Vector1_> {
  public:
    using BaseVariableType = VariableTime;

    virtual void setZero() override {
      this->_tainted = true;
      _estimate.setZero();
    }
    virtual void applyPerturbation(const Vector1f& pert) override {
      this->_tainted = true;
      _estimate += pert;
    }

    virtual ~VariableTime();
  };

} // namespace srrg2_solver
