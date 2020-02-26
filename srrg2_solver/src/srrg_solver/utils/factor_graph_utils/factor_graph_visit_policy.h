#pragma once
#include "factor_graph_visit_cost.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  class FactorGraphVisitPolicyBase : public Configurable {
  public:
    PARAM(PropertyInt, var_from_pos, "from var in factor, -1 means all", -1, 0);
    PARAM(PropertyInt, var_to_pos, "to var in factor, -1 means all", -1, 0);
    PARAM(PropertyConfigurable_<FactorGraphVisitCostBase>,
          cost_function,
          "cost function to evaluate the edge cost",
          0,
          0);

    virtual bool
    match(const FactorBase& factor, int var_from_pos, int var_to_pos);

    inline float
    cost(FactorBase& factor, int var_from_pos, int var_to_pos) {
      if (!match(factor, var_from_pos, var_to_pos))
        return -1;
      return param_cost_function->cost(
        factor, var_from_pos, var_to_pos);
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  class FactorGraphVisitPolicyByType : public FactorGraphVisitPolicyBase {
  public:
    using BaseType = FactorGraphVisitPolicyBase;
    using ThisType = FactorGraphVisitPolicyByType;
    PARAM(PropertyString, factor_classname, "factor class name", "none", 0);

    bool match(const FactorBase& factor,
               int var_from_pos,
               int var_to_pos) override;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace srrg2_solver
