#include "variable_point2.h"
#include <srrg_solver/solver_core/instance_macros.h>
#include "srrg_solver/solver_core/variable_impl.cpp"

namespace srrg2_solver {
  void VariablePoint2::setZero() {
    this->_tainted=true;
    _estimate.setZero();
  }

  void VariablePoint2::applyPerturbation(const Vector2f& pert) {
    this->_tainted=true;
    _estimate += pert;
  }

  INSTANTIATE(VariablePoint2)
  
} // namespace srrg2_solver
