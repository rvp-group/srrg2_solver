#include "factor_graph_visit_entry.h"
#include "srrg_solver/solver_core/factor_base.h"

namespace srrg2_solver {

  VariableVisitEntry::VariableVisitEntry(FactorBase* factor_,
                                         int parent_pos_,
                                         int var_pos_,
                                         float cost_) {
    expand(factor_, parent_pos_, var_pos_, cost_);
  }

  void
  VariableVisitEntry::expand(FactorBase* factor_, int parent_pos_, int var_pos_, float cost_) {
    factor     = factor_;
    parent_pos = parent_pos_;
    var_pos    = var_pos_;
    cost       = cost_;
    if (!factor) {
      var_pos = parent_pos_ = 0;
      variable = parent = 0;
      return;
    }
    variable = factor->variable(var_pos);
    parent   = factor->variable(parent_pos);
    ++num_visits;
    // std::cout<< __PRETTY_FUNCTION__ << ": ["
    //          << "v=" << variable->graphId() << ", "
    //          << "vt= " << (variable?variable->className():"") << ", "
    //          << "f=" << (factor?factor->graphId():-1) << ", "
    //          << "ft= " << (factor?factor->className():"") << ", "
    //          << "num_visits= " << num_visits
    //          << "cost= " << cost
    //          << "]" << std::endl;
  }

  VariableVisitEntry* VariableVisitEntryContainer::at(VariableBase::Id id) {
    auto it = _entries.find(id);
    if (it != _entries.end())
      return it->second.get();
    return 0;
  }

  void VariableVisitEntryContainer::add(VariableVisitEntry* entry) {
    // std::cerr << "VariableVisitEntryContainer::add| Adding: "
    //           << entry->variable->graphId();
    assert(!_entries.count(entry->variable->graphId()));
    //
    // std::cerr << " done" << std::endl;
    _entries.insert(std::make_pair(entry->variable->graphId(), VariableVisitEntryPtr(entry)));
  }

  void VariableVisitEntryContainer::clear() {
    _entries.clear();
  }

} // namespace srrg2_solver
