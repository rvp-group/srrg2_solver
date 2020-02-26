#pragma once
#include "factor_graph_visit_policy.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  class FactorGraphVisit : public Configurable {
  public:
    using BaseType = Configurable;
    using ThisType = FactorGraphVisit;

    PARAM_VECTOR(
      PropertyConfigurableVector_<FactorGraphVisitPolicyBase>,
      cost_policies,
      "the policies used to compute the cost of the factors during the visit",
      0);

    float cost(FactorBase& factor, int var_pos, int parent_pos);

    inline void setGraph(FactorGraphInterface& graph_) {
      _graph = &graph_;
    }

    inline void setSources(const std::vector<VariableBase::Id>& sources_) {
      _sources = &sources_;
    }
    void compute();
    inline VariableVisitEntryContainer& entries() {
      return _entries;
    }

  protected:
    FactorGraphVisitEntryQueue _queue;
    const std::vector<VariableBase::Id>* _sources = 0;
    FactorGraphInterface* _graph                  = 0;
    VariableVisitEntryContainer _entries;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace srrg2_solver
