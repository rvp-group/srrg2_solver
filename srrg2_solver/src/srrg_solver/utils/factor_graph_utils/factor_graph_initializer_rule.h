#pragma once
#include "srrg_solver/solver_core/factor_graph.h"
#include <queue>

namespace srrg2_solver {

  struct FactorGraphInitializer;

  struct FactorGraphInitializerRuleBase {
    FactorGraphInitializerRuleBase(FactorGraphInitializer* initializer_);
    virtual ~FactorGraphInitializerRuleBase() {
    }
    virtual bool init(VariableBase* variable_) = 0;

  protected:
    FactorGraphInitializer* _initializer;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  template <typename VariableType_>
  struct FactorGraphInitializerRule_ : public FactorGraphInitializerRuleBase {
    using VariableType = VariableType_;
    FactorGraphInitializerRule_(FactorGraphInitializer* initializer_) :
      FactorGraphInitializerRuleBase(initializer_) {
    }

    bool init(VariableBase* variable_) override {
      VariableType* v = dynamic_cast<VariableType*>(variable_);
      if (!v)
        return false;
      return doInit(v);
    }

    virtual bool doInit(VariableType* variable_) = 0;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using FactorGraphInitializerRulePtr =
    std::unique_ptr<FactorGraphInitializerRuleBase>;
  using FactorGraphInitializerRulePtrContainer =
    std::list<FactorGraphInitializerRulePtr>;

} // namespace srrg2_solver
