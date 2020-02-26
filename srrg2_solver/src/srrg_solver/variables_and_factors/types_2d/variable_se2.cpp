#include "variable_se2.h"


//! include this: this contains all the implementations of the factors
//! that are hidden to the modules that do not need them to avoid excessive compilation times (EVIL)
#include "srrg_solver/solver_core/variable_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  INSTANTIATE(VariableSE2Right)
  INSTANTIATE(VariableSE2Left)

} // namespace srrg2_solver
