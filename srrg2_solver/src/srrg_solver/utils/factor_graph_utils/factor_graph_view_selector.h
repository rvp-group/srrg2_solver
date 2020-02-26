#pragma once
#include "srrg_solver/solver_core/factor_graph.h"
namespace srrg2_solver {

  class FactorGraphViewSelector {
  public:
    // creates a view of the factors based on the variable ids
    // factors that connect *all* variables contained in the set
    // are added to the pool
    static void
    selectByVariables(FactorGraphView& dest,
                      FactorGraphInterface& src,
                      const std::vector<VariableBase::Id>& variable_ids);

    static void
    selectByFactors(FactorGraphView& dest,
                    FactorGraphInterface& src,
                    const std::vector<FactorBase::Id>& factor_ids);

    static void updateByVariable(FactorGraphView& dest,
                                 FactorGraphInterface& src,
                                 const VariableBase::Id& variable_id);

    static void updateByFactor(FactorGraphView& dest,
                               FactorGraphInterface& src,
                               const FactorBase::Id& factor_id);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace srrg2_solver
