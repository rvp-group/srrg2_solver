#pragma once
#include "factor_graph_initializer_rule.h"
#include "factor_graph_visit_entry.h"
#include "srrg_solver/solver_core/factor_graph.h"
#include <list>

namespace srrg2_solver {
  using namespace srrg2_core;

  struct FactorGraphInitializer : public Configurable {
    using BaseType = Configurable;
    using ThisType = FactorGraphInitializer;

    FactorGraphInitializer();
    void setGraph(FactorGraphInterface& graph_);
    void compute();

    FactorGraphInitializerRulePtrContainer _rules;
    VariableVisitEntryContainer _entries;
    FactorGraphInterface* _graph = 0;
    FactorGraphVisitEntryQueue _queue;
    bool isInit(VariableBase* variable);

  protected:
    bool initVariable(VariableBase* variable);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace srrg2_solver
