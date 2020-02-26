#include "instances.h"
#include "factor_graph.h"
#include "iteration_algorithm_gn.h"
#include "iteration_algorithm_lm.h"
#include "solver.h"
#include "robustifier_policy.h"
#include "termination_criteria.h"

namespace srrg2_solver {

  void solver_registerTypes() {
    BOSS_REGISTER_CLASS(RobustifierSaturated);
    BOSS_REGISTER_CLASS(RobustifierCauchy);
    BOSS_REGISTER_CLASS(RobustifierClamp);
    BOSS_REGISTER_CLASS(RobustifierPolicyByType);
    BOSS_REGISTER_CLASS(Solver);
    BOSS_REGISTER_CLASS(SimpleTerminationCriteria);
    BOSS_REGISTER_CLASS(GradientNormTerminationCriteria);
    BOSS_REGISTER_CLASS(PerturbationNormTerminationCriteria);
    BOSS_REGISTER_CLASS(IterationAlgorithmGN);
    BOSS_REGISTER_CLASS(IterationAlgorithmLM);
    BOSS_REGISTER_CLASS(FactorGraph);
  }
} // namespace srrg2_solver
