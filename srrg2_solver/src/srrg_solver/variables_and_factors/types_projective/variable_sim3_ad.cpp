#include "variable_sim3_ad.h"
#include "srrg_solver/solver_core/instance_macros.h"
#include "srrg_solver/solver_core/variable_impl.cpp"

namespace srrg2_solver {
  INSTANTIATE(VariableSim3EulerRightAD)
  INSTANTIATE(VariableSim3EulerLeftAD)
  INSTANTIATE(VariableSim3QuaternionRightAD)
  INSTANTIATE(VariableSim3QuaternionLeftAD)
  
} // namespace srrg2_solver
