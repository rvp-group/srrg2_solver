#include "factor_graph_visit.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  float FactorGraphVisit::cost(FactorBase& factor, int parent_pos, int var_pos) {
    for (size_t i = 0; i < param_cost_policies.size(); ++i) {
      FactorGraphVisitPolicyBase* policy = param_cost_policies[i].get();
      float c                            = policy->cost(factor, parent_pos, var_pos);
      if (c > 0) {
        return c;
      }
    }
    return -1;
  }

  void FactorGraphVisit::compute() {
    if (!_graph) {
      throw std::runtime_error("FactorGraphVisit::compute|graph not set");
    }

    if (!_sources) {
      throw std::runtime_error("FactorGraphVisit::compute|sources not set");
    }

    _entries.clear();
    IdVariablePtrContainer& variables = _graph->variables();
    for (IdVariablePtrContainer::iterator it = variables.begin(); it != variables.end(); ++it) {
      VariableBase* v = it.value();
      _entries.add(new VariableVisitEntry(v));
    }
    _queue = FactorGraphVisitEntryQueue();

    for (auto it = _sources->begin(); it != _sources->end(); ++it) {
      VariableVisitEntry* e = _entries.at(*it);
      assert(e && "source not in graph");
      e->cost = 0;
      _queue.push(e);
    }

    while (!_queue.empty()) {
      VariableVisitEntry* e = _queue.top();
      _queue.pop();
      VariableBase* v = e->variable;
      // std::cerr << std::endl;
      // std::cerr << std::endl;
      // std::cerr << "EXPANDING: " << v->graphId() << std::endl;
      float v_cost = e->cost;
      // scan all factors
      auto f_lower = _graph->lowerFactor(v);
      auto f_upper = _graph->upperFactor(v);
      for (auto it = f_lower; it != f_upper; ++it) {
        int v_pos          = -1;
        FactorBase* f = it->second;
        if (!f->enabled())
          continue;

        // find index of var in factor
        for (int i = 0; i < f->numVariables(); ++i) {
          if (f->variable(i) == v) {
            e->var_pos = i;
            v_pos      = i;
            break;
          }
        }

        // std::cerr << "f: " << f << std::endl;
        if (!e->factor) {
          e->factor = f;
          assert(v == f->variable(e->var_pos) && "bookkeping error 1"); // ia dunno if it's this
        }

        // scan all variables
        for (int nv_pos = 0; nv_pos < f->numVariables(); ++nv_pos) {
          VariableBase* nv = f->variable(nv_pos);
          if (nv == v)
            continue;

          VariableVisitEntry* ne = _entries.at(nv->graphId());
          assert(ne && "bookkeeping error 2");
          float d_cost = cost(*f, v_pos, nv_pos);
          // std::cerr << "nv" << nv->graphId() << std::endl;
          // std::cerr << "d_cost=" << d_cost << std::endl;
          // std::cerr << "v_cost=" << v_cost << std::endl;
          // std::cerr << "ne_cost=" << ne->cost << std::endl;
          if (d_cost < 0) {
            continue;
          }
          if (v_cost + d_cost < ne->cost) {
            //            std::cerr << "cost: " << d_cost << std::endl;
            ne->expand(f, v_pos, nv_pos, v_cost + d_cost);
            _queue.push(ne);
          }
        }
      }
    }
  }

} // namespace srrg2_solver
