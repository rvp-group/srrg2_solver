#include "variable_time_ad.h"
#include "srrg_solver/solver_core/variable_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  VariableTimeAD::~VariableTimeAD(){}
  INSTANTIATE(VariableTimeAD)
}
