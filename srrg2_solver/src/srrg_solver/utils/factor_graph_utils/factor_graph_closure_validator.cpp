#include "factor_graph_closure_validator.h"
#include "srrg_solver/variables_and_factors/types_2d/variable_se2.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3.h"
#include <fstream>

#include "srrg_solver/variables_and_factors/types_2d/se2_pose_pose_geodesic_error_factor.h"
#include "srrg_solver/variables_and_factors/types_3d/se3_pose_pose_geodesic_error_factor.h"

std::ofstream null_stream = std::ofstream("/dev/null");
// std::ostream& console_stream(std::cerr);
std::ostream& console_stream(null_stream);

namespace srrg2_solver {
  using namespace std;

  void FactorGraphClosureValidator::resetClosureStatus() {
    ClosureStats r;
    for (auto& it : _closures) {
      it.second = r;
    }
  }

  void FactorGraphClosureValidator::addConstant(VariableBase* constant) {
    _constants.insert(constant);
  }

  bool FactorGraphClosureValidator::isConstant(VariableBase* constant) const {
    return _constants.count(constant);
  }

  void FactorGraphClosureValidator::addClosure(FactorBasePtr factor) {
    _ptr_sptr_map.insert(std::make_pair(factor.get(), factor));
    _closures.insert(std::make_pair(factor.get(), ClosureStats()));
  }

  void FactorGraphClosureValidator::removeClosure(FactorBasePtr factor) {
    auto it = _closures.find(factor.get());
    if (it == _closures.end()) {
      return;
    }
    _closures.erase(it);
    _ptr_sptr_map.erase(factor.get());
  }

  void FactorGraphClosureValidator::setGraph(FactorGraphInterfacePtr graph_) {
    if (_graph == graph_) {
      return;
    }
    _graph = graph_;
    _closures.clear();
    _updated_closures.clear();
  }

  void FactorGraphClosureValidator::compute(VariableBase* active_variable) {
    // resetClosureStatus();
    preparePartitioning(active_variable);
    computePartitions();
    clusterClosures(active_variable);
    for (auto it = _updated_closures.begin(); it != _updated_closures.end(); ++it) {
      ClosureStats* stats = it->second;
      ++stats->num_rounds_checked;
      if (stats->status != ClosureStats::Pending) {
        continue;
      }
      if (stats->num_times_checked < param_min_times_checked.value()) {
        stats->status = ClosureStats::Pending;
        continue;
      }
      float inlier_ratio = (float) stats->num_times_good / (float) stats->num_times_checked;
      if (inlier_ratio < param_inlier_reject_ratio.value()) {
        stats->status = ClosureStats::Rejected;
        continue;
      }
      if (inlier_ratio > param_inlier_accept_ratio.value()) {
        stats->status = ClosureStats::Accepted;
        continue;
      }
    }
  }

  void FactorGraphClosureValidator::preparePartitioning(VariableBase* active_variable) {
    partitions.clear();
    var_to_partition_map.clear();
    open_variables.clear();
    _updated_closures.clear();

    // scan all closures where one variable is active
    std::set<VariableBase*> active_variables;
    for (auto it : _closures) {
      FactorBase* f = it.first;
      bool add_variable  = false;
      for (int v_pos = 0; v_pos < f->numVariables(); ++v_pos) {
        VariableBase* v = f->variable(v_pos);
        if (v == active_variable) {
          add_variable = true;
        }
      }
      if (!add_variable)
        continue;
      _updated_closures[_ptr_sptr_map[f]] = &it.second;
      for (int v_pos = 0; v_pos < f->numVariables(); ++v_pos) {
        VariableBase* v = f->variable(v_pos);
        open_variables.insert(v);
      }
    }
  }

  void FactorGraphClosureValidator::joinPartitions(FactorGraphView* dest, FactorGraphView* src) {
    // console_stream << "Joining" << dest << " " << src << endl;
    for (auto it = src->variables().begin(); it != src->variables().end(); ++it) {
      VariableBase* variable = it.value();
      dest->_variables.insert(std::make_pair(variable->graphId(), variable));
      if (!isConstant(it.value())) {
        var_to_partition_map[variable] = dest;
      }
    }
  }

  FactorGraphView* FactorGraphClosureValidator::addVariable(FactorGraphView* partition_,
                                                            VariableBase* variable) {
    FactorGraphView* other_partition = partition(variable);
    if (other_partition == partition_) {
      return partition_;
    }
    if (other_partition == 0) {
      partition_->_variables.insert(std::make_pair(variable->graphId(), variable));
      if (!isConstant(variable)) {
        var_to_partition_map[variable] = partition_;
      }
      return partition_;
    }
    joinPartitions(other_partition, partition_);
    auto p_it       = partitions.find(other_partition);
    auto erase_p_it = partitions.find(partition_);
    partitions.erase(erase_p_it);
    return p_it->first;
  }

  FactorGraphView* FactorGraphClosureValidator::partition(VariableBase* variable) {
    auto it = var_to_partition_map.find(variable);
    if (it == var_to_partition_map.end()) {
      return nullptr;
    }
    return it->second;
  }

  struct QEntry {
    QEntry(VariableBase* var_, float cost_ = 0) : variable(var_), cost(cost_) {
    }
    VariableBase* variable;
    float cost;
    inline bool operator<(const QEntry& other) {
      return cost > other.cost;
    }
  };

  void FactorGraphClosureValidator::expandPartition(VariableBase* active_variable) {
    FactorGraphView* new_partition = new FactorGraphView;
    partitions.insert(std::make_pair(new_partition, FactorGraphViewPtr(new_partition)));

    console_stream << __PRETTY_FUNCTION__ << ": " << active_variable->graphId() << endl;
    std::map<VariableBase*, float> costs;
    std::deque<QEntry> queue;
    costs.insert(std::make_pair(active_variable, 0.f));
    queue.push_back(QEntry(active_variable, 0));
    new_partition = addVariable(new_partition, active_variable);

    while (!queue.empty()) {
      QEntry expanded = queue.front();
      queue.pop_front();
      if (expanded.cost > param_partition_expansion_range.value())
        continue;

      auto f_it    = _graph->lowerFactor(expanded.variable);
      auto f_upper = _graph->upperFactor(expanded.variable);
      for (; f_it != f_upper; ++f_it) {
        FactorBase* f = f_it->second;
        // if the factor is a closure, we ignore the expansion
        if (_closures.find(f) != _closures.end())
          continue;

        // scan all variables
        for (int nv_pos = 0; nv_pos < f->numVariables(); ++nv_pos) {
          VariableBase* nv = f->variable(nv_pos);
          if (nv == expanded.variable)
            continue;
          if (isConstant(nv)) {
            new_partition = addVariable(new_partition, nv);
            continue;
          }
          // retrieve new cost
          float nv_cost = std::numeric_limits<float>::max();
          auto cost_it  = costs.find(nv);
          if (cost_it == costs.end()) {
            auto ins_result = costs.insert(std::make_pair(nv, nv_cost));
            cost_it         = ins_result.first;
          } else {
            nv_cost = cost_it->second;
          }

          // conditional expansion

          float nv_new_cost = expanded.cost + computeTraversalCost(expanded.variable, nv);
          console_stream << "expanding: " << nv->graphId() << " cost: " << nv_cost << " new_cost"
                         << nv_new_cost << endl;

          if ((nv_new_cost < nv_cost) && (nv_new_cost < param_partition_expansion_range.value())) {
            console_stream << "expanded!" << std::endl;
            // adding and merging to current partition
            new_partition   = addVariable(new_partition, nv);
            cost_it->second = nv_new_cost;
            queue.push_back(QEntry(nv, nv_new_cost));
          }
        }
      }
    }
  }

  float FactorGraphClosureValidator::computeTraversalCost(VariableBase* v1_,
                                                          VariableBase* v2_) {
    //  SE2
    {
      VariableSE2Base* v1 = dynamic_cast<VariableSE2Base*>(v1_);
      VariableSE2Base* v2 = dynamic_cast<VariableSE2Base*>(v2_);
      if (v1 && v2) {
        return (v1->estimate().translation() - v2->estimate().translation()).norm();
      }
    }

    // SE3
    {
      VariableSE3Base* v1 = dynamic_cast<VariableSE3Base*>(v1_);
      VariableSE3Base* v2 = dynamic_cast<VariableSE3Base*>(v2_);
      if (v1 && v2) {
        return (v1->estimate().translation() - v2->estimate().translation()).norm();
      }
    }
    return std::numeric_limits<float>::max();
  }

  void FactorGraphClosureValidator::computePartitions() {
    while (!open_variables.empty()) {
      VariableBase* root_v = *open_variables.begin();
      open_variables.erase(open_variables.begin());
      expandPartition(root_v);
    }
    console_stream << __PRETTY_FUNCTION__ << "| partitions.size(): " << partitions.size()
                   << " partitions" << endl;
  }

  using PMapKey = std::set<FactorGraphView*>;
  struct PMapKeyCompare {
    inline bool operator()(const PMapKey& a, const PMapKey& b) const {
      if (a.size() != b.size()) {
        return a.size() < b.size();
      }
      auto it_a = a.begin();
      auto it_b = b.begin();
      while (it_a != a.end()) {
        if (*it_a < *it_b) {
          return true;
        }
        ++it_a;
        ++it_b;
      }
      return false;
    }
  };

  using PMapKeyClosureMap = std::map<PMapKey, std::list<FactorBase*>, PMapKeyCompare>;

  void FactorGraphClosureValidator::clusterClosures(VariableBase* active_variable) {
    _updated_closures.clear();
    FactorGraphView* active_partition = partition(active_variable);
    if (!active_partition) {
      return;
    }
    // group all closures that connect the same partitions
    PMapKeyClosureMap closure_map;
    for (auto cl_it = _closures.begin(); cl_it != _closures.end(); ++cl_it) {
      FactorBase* f = cl_it->first;
      PMapKey key;
      bool is_active = false;
      for (int nv_pos = 0; nv_pos < f->numVariables(); ++nv_pos) {
        VariableBase* nv = f->variable(nv_pos);
        if (isConstant(nv)) {
          continue;
        }
        FactorGraphView* p = partition(nv);
        if (!p) {
          is_active = false;
          break;
        }
        key.insert(p);
        if (p == active_partition) {
          is_active = true;
        }
      }
      if (!is_active) {
        continue;
      }
      auto it = closure_map.find(key);
      if (it == closure_map.end()) {
        std::pair<PMapKeyClosureMap::iterator, bool> result =
          closure_map.insert(std::make_pair(key, std::list<FactorBase*>()));
        it = result.first;
      }
      it->second.push_back(f);
    }
    console_stream << __PRETTY_FUNCTION__ << "| number of clusters: " << closure_map.size() << endl;
    int count = 0;
    for (auto it = closure_map.begin(); it != closure_map.end(); ++it) {
      console_stream << "Cluster[" << count << "]: partitions: [";
      for (auto& p : it->first) {
        console_stream << p << " ";
      }
      console_stream << "] closures: [";
      for (auto& c : it->second) {
        console_stream << c << " ";
      }
      console_stream << "]" << endl;
      ++count;
    }

    // check clusters
    for (auto it = closure_map.begin(); it != closure_map.end(); ++it) {
      console_stream << "Cluster[" << count << "] " << endl;
      if (it->first.size() == 1) {
        doValidate1(active_partition, it->second);
      }
      if (it->first.size() == 2) {
        auto f_it                = it->first.begin();
        FactorGraphView* first_p = *f_it;
        ++f_it;
        FactorGraphView* second_p        = *f_it;
        FactorGraphView* other_partition = first_p == active_partition ? second_p : first_p;
        // if (other_partition)
        doValidate2(active_partition, other_partition, it->second);
      }
      ++count;
    }
  }

  void FactorGraphClosureValidator::pushAll(FactorGraphView* graph) {
    for (auto it = graph->variables().begin(); it != graph->variables().end(); ++it) {
      VariableBase* v = it.value();
      if (!isConstant(v)) {
        v->push();
      }
    }
  }

  void FactorGraphClosureValidator::popAll(FactorGraphView* graph) {
    for (auto it = graph->variables().begin(); it != graph->variables().end(); ++it) {
      VariableBase* v = it.value();
      if (!isConstant(v)) {
        v->pop();
      }
    }
  }

  void FactorGraphClosureValidator::doValidate1(FactorGraphView* active_partition,
                                                std::list<FactorBase*>& factors) {
    console_stream << __PRETTY_FUNCTION__ << "| active_partition: " << active_partition << endl;
    for (auto f : factors) {
      f->compute(true, true);
      console_stream << "validate 1: F: " << f << " chi: " << f->stats().chi << endl;
      auto it = _closures.find(f);
      assert(it != _closures.end() && "closure not in list, fatal");
      ClosureStats& stats = it->second;
      stats.num_times_checked++;
      if (f->stats().chi < param_inlier_chi.value()) {
        stats.num_times_good++;
        stats.status = ClosureStats::Accepted; // within a local map, we decide stright away
      } else {
        stats.status = ClosureStats::Rejected;
      }
      _updated_closures[_ptr_sptr_map[f]] = &it->second;
    }
  }

  bool FactorGraphClosureValidator::doFullfillSE2(VariableBase* fixed_var_,
                                                  FactorBase* factor_,
                                                  FactorGraphView* partition) {
    VariableSE2Base* fixed_var       = dynamic_cast<VariableSE2Base*>(fixed_var_);
    SE2PosePoseGeodesicErrorFactor* f = dynamic_cast<SE2PosePoseGeodesicErrorFactor*>(factor_);
    if (!fixed_var | !f) {
      return false;
    }
    // compute how much the partition should be moved
    bool direct                = f->variable(0) == fixed_var;
    Isometry2f total_t         = Eigen::Isometry2f::Identity();
    Isometry2f target_pose     = Eigen::Isometry2f::Identity();
    VariableSE2Base* other_var = 0;
    if (direct) {
      other_var   = dynamic_cast<VariableSE2Base*>(f->variable(1));
      target_pose = fixed_var->estimate() * f->measurement();
    } else {
      other_var   = dynamic_cast<VariableSE2Base*>(f->variable(0));
      target_pose = fixed_var->estimate() * f->measurement().inverse();
    }
    total_t = target_pose * other_var->estimate().inverse();
    for (auto it = partition->variables().begin(); it != partition->variables().end(); ++it) {
      VariableBase* v = it.value();
      if (isConstant(v)) {
        continue;
      }
      VariableSE2Base* moved_v = dynamic_cast<VariableSE2Base*>(v);
      if (!moved_v) {
        continue;
      }
      moved_v->setEstimate(total_t * moved_v->estimate());
    }
    return true;
  }

  bool FactorGraphClosureValidator::doFullfillSE3(VariableBase* fixed_var_,
                                                  FactorBase* factor_,
                                                  FactorGraphView* partition) {
    VariableSE3Base* fixed_var       = dynamic_cast<VariableSE3Base*>(fixed_var_);
    SE3PosePoseGeodesicErrorFactor* f = dynamic_cast<SE3PosePoseGeodesicErrorFactor*>(factor_);
    if (!fixed_var | !f) {
      return false;
    }
    // compute how much the partition should be moved
    bool direct                = f->variable(0) == fixed_var;
    Isometry3f total_t         = Eigen::Isometry3f::Identity();
    Isometry3f target_pose     = Eigen::Isometry3f::Identity();
    VariableSE3Base* other_var = 0;
    if (direct) {
      other_var   = dynamic_cast<VariableSE3Base*>(f->variable(1));
      target_pose = fixed_var->estimate() * f->measurement();
    } else {
      other_var   = dynamic_cast<VariableSE3Base*>(f->variable(0));
      target_pose = fixed_var->estimate() * f->measurement().inverse();
    }
    total_t = target_pose * other_var->estimate().inverse();
    for (auto it = partition->variables().begin(); it != partition->variables().end(); ++it) {
      VariableBase* v = it.value();
      if (isConstant(v)) {
        continue;
      }
      VariableSE3Base* moved_v = dynamic_cast<VariableSE3Base*>(v);
      if (!moved_v) {
        continue;
      }
      moved_v->setEstimate(total_t * moved_v->estimate());
    }
    return true;
  }

  bool FactorGraphClosureValidator::doFullfill(VariableBase* fixed_variable,
                                               FactorBase* factor,
                                               FactorGraphView* partition) {
    if (doFullfillSE2(fixed_variable, factor, partition)) {
      return true;
    }
    if (doFullfillSE3(fixed_variable, factor, partition)) {
      return true;
    }
    return false;
  }

  void FactorGraphClosureValidator::doValidate2(FactorGraphView* active_partition,
                                                FactorGraphView* other_partition,
                                                std::list<FactorBase*>& factors) {
    console_stream << __PRETTY_FUNCTION__ << "| active_partition: " << active_partition
                   << "  other_partition: " << other_partition << endl;
    // save graph
    pushAll(other_partition);
    for (auto out_f : factors) {
      console_stream << "validate 2: F: " << out_f << endl;
      VariableBase* active_var = 0;
      VariableBase* v0         = out_f->variable(0);
      VariableBase* v1         = out_f->variable(1);
      if (active_partition->variable(v0->graphId())) {
        active_var = v0;
      }
      if (active_partition->variable(v1->graphId())) {
        active_var = v1;
      }
      assert(active_var && "! active var");
      if (!doFullfill(active_var, out_f, other_partition)) {
        continue;
      }
      for (auto in_f : factors) {
        in_f->compute(true, true);
        console_stream << " if " << in_f << " chi: " << in_f->stats().chi << endl;

        auto it = _closures.find(in_f);
        assert(it != _closures.end() && "closure not in list, fatal");
        ClosureStats& stats = it->second;
        stats.num_times_checked++;
        if (in_f->stats().chi < param_inlier_chi.value()) {
          stats.num_times_good++;
        }
        _updated_closures[_ptr_sptr_map[in_f]] = &it->second;
      }
    }
    // restore graph
    popAll(other_partition);
  }

} // namespace srrg2_solver
