#include "factor_graph_view_selector.h"

namespace srrg2_solver {

  void
  FactorGraphViewSelector::selectByVariables(FactorGraphView& dest,
                                             FactorGraphInterface& src,
                                             const std::vector<VariableBase::Id>& variable_ids) {
    dest.clear();
    // we add to the variables of the view all variables in vector
    for (auto graph_id : variable_ids) {
      VariableBase* v = src.variable(graph_id);
      if (!v) {
        throw std::runtime_error("variable not in graph");
      }
      dest._variables.insert(std::make_pair(graph_id, v));
      auto lower = src.lowerFactor(v);
      auto upper = src.upperFactor(v);
      while (lower != upper) {
        FactorBase* f = lower->second;
        ++lower;
        // check that all variables belong to the set
        for (int pos = 0; pos < f->numVariables(); ++pos) {
          if (!f->variable(pos)) {
            continue;
          }
        }
        dest._factors.insert(std::make_pair(f->graphId(), f));
      }
    }
    dest.bindFactors();
    // we add to the factors all factor whose variables are all in the list of variables
  }

  void
  FactorGraphViewSelector::selectByFactors(FactorGraphView& dest,
                                           FactorGraphInterface& src,
                                           const std::vector<FactorBase::Id>& factor_ids) {
    // we add to the factors all factors in vectors;
    // we add to the variables all variables connected by the factors
    dest.clear();
    // we add to the variables of the view all variables in vector
    for (auto graph_id : factor_ids) {
      FactorBase* f = src.factor(graph_id);
      if (!f) {
        throw std::runtime_error("factor not in graph");
      }
      dest._factors.insert(std::make_pair(graph_id, f));
      for (int pos = 0; pos < f->numVariables(); ++pos) {
        VariableBase* v = f->variable(pos);
        if (v) {
          dest._variables.insert(std::make_pair(v->graphId(), v));
        }
      }
    }
    dest.bindFactors();
  }

  void FactorGraphViewSelector::updateByVariable(FactorGraphView& dest,
                                                 FactorGraphInterface& src,
                                                 const VariableBase::Id& variable_id) {
    VariableBase* v = src.variable(variable_id);
    if (!v) {
      throw std::runtime_error("variable not in graph");
    }
    dest._variables.insert(std::make_pair(variable_id, v));
  }

  void FactorGraphViewSelector::updateByFactor(FactorGraphView& dest,
                                               FactorGraphInterface& src,
                                               const FactorBase::Id& factor_id) {
    FactorBase* f = src.factor(factor_id);
    if (!f) {
      throw std::runtime_error("factor not in graph");
    }
    dest._factors.insert(std::make_pair(factor_id, f));
    for (int pos = 0; pos < f->numVariables(); ++pos) {
      VariableBase* v = f->variable(pos);
      if (v) {
        dest._variables.insert(std::make_pair(v->graphId(), v));
      }
    }
    dest.bindFactor(f);
  }

} // namespace srrg2_solver
