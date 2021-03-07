#include "factor_graph.h"
#include <srrg_boss/deserializer.h>
#include <srrg_boss/serializer.h>
// included here since I instantiate all methods
namespace srrg2_solver {

  using namespace std;
  using namespace srrg2_core;

  FactorGraphInterface::~FactorGraphInterface() {
  }

  void FactorGraphInterface::clear() {
    variables().clear();
    factors().clear();
    _variable_factor_map.clear();
  }

  void FactorGraphInterface::printVariables() {
    for (auto it = variables().begin(); it != variables().end(); ++it) {
      const VariableBase* const v = it.value();
      cerr << "id: " << v->graphId() << " " << v->status() << endl;
    }
  }

  VariableBase* FactorGraphInterface::variable(VariableBase::Id id) {
    auto it = variables().find(id);
    if (it == variables().end()) {
      return 0;
    }
    VariableBase* v = it.value();
    return v;
  }

  FactorBase* FactorGraphInterface::factor(FactorBase::Id id) {
    auto it = factors().find(id);
    if (it == factors().end()) {
      return 0;
    }
    FactorBase* f = it.value();
    return f;
  }

  int FactorGraphInterface::bindFactor(FactorBase* f) {
    int num_vars = 0;
    num_vars     = f->bind(variables());
    for (int var_idx = 0; var_idx < f->numVariables(); ++var_idx) {
      VariableBase* v = f->variable(var_idx);
      if (v) {
        _variable_factor_map.insert(std::make_pair(v, f));
      }
    }
    // we now populate the graph from scratch
    return num_vars;
  }

  void FactorGraphInterface::unbindFactor(FactorBase* f) {
    for (int var_idx = 0; var_idx < f->numVariables(); ++var_idx) {
      VariableBase* v = f->variable(var_idx);
      f->setVariable(var_idx, 0);
      if (v) {
        auto it_start = _variable_factor_map.lower_bound(v);
        auto it_end   = _variable_factor_map.upper_bound(v);
        while (it_start != it_end) {
          if (it_start->second == f) {
            _variable_factor_map.erase(it_start);
            return;
          }
          ++it_start;
        }
      }
    }
  }

  int FactorGraphInterface::bindFactors() {
    //    std::cerr << "BindFactors " << this << std::endl;
    _variable_factor_map.clear();
    int unbinded_vars = 0;
    for (auto it = factors().begin(); it != factors().end(); ++it) {
      FactorBase* f = it.value();
      unbinded_vars += bindFactor(f);
    }
    if (unbinded_vars > 0) {
      cerr << "there are dangling variables\n" << endl;
      cerr << "no optimization possible\n" << endl;
      cerr << "either: " << endl;
      cerr << "- instantiate the variables in the graph" << endl;
      cerr << "- set the create_on_bind parameter to true" << endl;
    }
    // we now populate the graph from scratch
    return unbinded_vars;
  }

  FactorGraphInterface::VariableFactorMapIterator
  FactorGraphInterface::lowerFactor(const VariableBase* variable) {
    return _variable_factor_map.lower_bound(variable);
  }

  FactorGraphInterface::VariableFactorMapIterator
  FactorGraphInterface::upperFactor(const VariableBase* variable) {
    return _variable_factor_map.upper_bound(variable);
  }

  FactorGraphView::~FactorGraphView() {
    clear();
  }

  IdVariablePtrContainer& FactorGraphView::variables() {
    return _variables;
  }

  IdFactorPtrContainer& FactorGraphView::factors() {
    return _factors;
  }

  FactorGraph::~FactorGraph() {
    clear();
  }

  IdVariablePtrContainer& FactorGraph::variables() {
    return _variables;
  }

  IdFactorPtrContainer& FactorGraph::factors() {
    return _factors;
  }

  void FactorGraph::serialize(ObjectData& odata, IdContext& context) {
    using namespace std;
    std::set<VariableBase::Id> selected_variables;
    std::vector<FactorBase*> selected_factors;
    for (auto it = factors().begin(); it != factors().end(); ++it) {
      FactorBase* f = it.value();
      if (f->level() != _level_serialization) {
        continue;
      }
      selected_factors.push_back(f);
      int num_var = f->numVariables();
      for (int v = 0; v < num_var; ++v) {
        selected_variables.insert(f->variableId(v));
      }
    }
    ArrayData* var_data = new ArrayData;
    for (auto it = selected_variables.begin(); it != selected_variables.end(); ++it) {
      VariableBase* v = this->variable(*it);
      var_data->addPointer(v);
    }
    odata.setField("variables", var_data);
    ArrayData* fact_data = new ArrayData;
    for (auto it = selected_factors.begin(); it != selected_factors.end(); ++it) {
      FactorBase* f = *it;
      fact_data->addPointer(f);
    }
    odata.setField("factors", fact_data);
  }

  bool FactorGraph::addVariable(VariableBasePtr var) {
    if (var->graphId() == -1) {
      var->setGraphId(_last_graph_id++);
    }
    VariableBase const* v = variable(var->graphId());
    if (v) {
      assert("variable already in graph");
      return false;
    }
    _variables.insert(std::make_pair(var->graphId(), var));
    return true;
  }

  bool FactorGraph::addFactor(FactorBasePtr fac, bool auto_bind) {
    if (fac->graphId() == -1) {
      fac->setGraphId(_last_graph_id++);
    }
    FactorBase const* f = factor(fac->graphId());
    if (f) {
      assert("factor already in graph");
      return false;
    }

    // ia check if factor is sick
    // gg: this is "vomito fecale"
    if (fac->numVariables()>1 && fac->variableId(0) == fac->variableId(1)) {
      std::cerr << "FactorGraph::addFactor|waring, ignoring factor between [" << fac->variableId(0)
                << " - " << fac->variableId(1) << "]\n";
      return false;
    }

    _factors.insert(std::make_pair(fac->graphId(), fac));
    if (auto_bind) {
      bindFactor(fac.get());
    }
    return true;
  }

  bool FactorGraph::removeFactor(FactorBasePtr f) {
    return removeFactor(f.get());
  }

  bool FactorGraph::removeFactor(FactorBase* f) {
    auto it = factors().find(f->graphId());
    if (it == factors().end()) {
      assert("factor not in graph");
      return false;
    }
    unbindFactor(f);
    factors().erase(it);
    return true;
  }

  void FactorGraph::deserialize(ObjectData& odata, IdContext& context) {
    _variables.clear();
    _factors.clear();
    ArrayData* var_data = dynamic_cast<ArrayData*>(odata.getField("variables"));
    for (auto it = var_data->begin(); it != var_data->end(); ++it) {
      ValueData* data = *it;
      VariableBasePtr v =
        std::dynamic_pointer_cast<VariableBase>(data->getPointer()->getSharedPtr());
      addVariable(v);
      _last_graph_id = std::max(v->graphId(), _last_graph_id);
    }

    ArrayData* fact_data = dynamic_cast<ArrayData*>(odata.getField("factors"));
    for (auto it = fact_data->begin(); it != fact_data->end(); ++it) {
      ValueData* data = *it;
      FactorBasePtr f =
        std::dynamic_pointer_cast<FactorBase>(data->getPointer()->getSharedPtr());
      addFactor(f);
      _last_graph_id = std::max(f->graphId(), _last_graph_id);
    }
    ++_last_graph_id;

    bindFactors();
  }

  // convenience function to write a factor graph to a file
  void FactorGraph::write(const std::string& filename) {
    using namespace std;
    bindFactors();
    std::set<VariableBase::Id> selected_variables;
    std::vector<FactorBase*> selected_factors;
    for (auto it = factors().begin(); it != factors().end(); ++it) {
      FactorBase* f = it.value();
      if (f->level() != _level_serialization) {
        continue;
      }
      selected_factors.push_back(f);
      int num_var = f->numVariables();
      for (int v = 0; v < num_var; ++v) {
        selected_variables.insert(f->variableId(v));
      }
    }

    Serializer ser;
    ser.setFilePath(filename);
    for (VariableBase::Id v : selected_variables) {
      VariableBase* variable = this->variable(v);
      ser.writeObject(*variable);
    }
    for (FactorBase* factor : selected_factors) {
      ser.writeObject(*factor);
    }
    ser.writeObject(*this);
  }

  // convenience functions a graph from a file
  FactorGraphPtr FactorGraph::read(const std::string& filename) {
    Deserializer des;
    des.setFilePath(filename);
    SerializablePtr o;
    while ((o = des.readObjectShared())) {
      FactorGraphPtr graph = std::dynamic_pointer_cast<FactorGraph>(o);
      if (graph) {
        return graph;
      }
    }
    return FactorGraphPtr();
  }

} // namespace srrg2_solver
