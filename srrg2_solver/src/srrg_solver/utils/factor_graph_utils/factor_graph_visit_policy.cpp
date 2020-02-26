#include "factor_graph_visit_policy.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  bool FactorGraphVisitPolicyBase::match(const FactorBase& factor,
                                         int var_from_pos_,
                                         int var_to_pos_) {
    if (param_var_from_pos.value() > -1 && param_var_from_pos.value() != var_from_pos_)
      return false;

    if (param_var_to_pos.value() > -1 && param_var_to_pos.value() != var_to_pos_)
      return false;
    return true;
  }

  bool FactorGraphVisitPolicyByType::match(const FactorBase& factor,
                                           int var_from_pos_,
                                           int var_to_pos_) {
    if (!FactorGraphVisitPolicyBase::match(factor, var_from_pos_, var_to_pos_))
      return false;
    if (factor.className() != param_factor_classname.value())
      return false;
    return true;
  }
} // namespace srrg2_solver
