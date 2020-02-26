#include "instances.h"
#include "factor_graph_closure_validator.h"
#include "factor_graph_initializer.h"
#include "factor_graph_view_selector.h"
#include "factor_graph_visit.h"

namespace srrg2_solver {
  void solver_utils_registerTypes() {
    BOSS_REGISTER_CLASS(FactorGraphVisit);
    BOSS_REGISTER_CLASS(FactorGraphVisitCostUniform);
    BOSS_REGISTER_CLASS(FactorGraphVisitPolicyBase);
    BOSS_REGISTER_CLASS(FactorGraphVisitPolicyByType);
    BOSS_REGISTER_CLASS(FactorGraphClosureValidator);
    BOSS_REGISTER_CLASS(FactorGraphInitializer);
  }
} // namespace srrg2_solver
