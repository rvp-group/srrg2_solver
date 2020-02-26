#include "variable_se3_ad.h"
#include "srrg_solver/solver_core/instance_macros.h"
#include "srrg_solver/solver_core/variable_impl.cpp"

namespace srrg2_solver {
  INSTANTIATE(VariableSE3EulerRightAD)
  INSTANTIATE(VariableSE3EulerLeftAD)
  INSTANTIATE(VariableSE3QuaternionRightAD)
  INSTANTIATE(VariableSE3QuaternionLeftAD)
  
} // namespace srrg2_solver
