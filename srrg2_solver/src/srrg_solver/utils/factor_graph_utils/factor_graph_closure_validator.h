#pragma once
#include "srrg_solver/solver_core/factor_graph.h"
#include <deque>
#include <set>
#include <srrg_config/configurable.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  class FactorGraphClosureValidator : public Configurable {
  public:
    PARAM(PropertyFloat,
          partition_expansion_range,
          "region around the endpoint of a closure to compute partitions",
          5,
          0);
    PARAM(PropertyInt,
          min_times_checked,
          "minimum number a closure is checked",
          5,
          0);
    PARAM(PropertyFloat,
          inlier_accept_ratio,
          "ratio to accept a closure after min checks",
          0.5,
          0);
    PARAM(PropertyFloat,
          inlier_reject_ratio,
          "ratio to reject a closure after min checks",
          0.3,
          0);
    PARAM(PropertyFloat, inlier_chi, "chi2 of an inlier", 0.05, 0);

  public:
    // set a graph (once)
    void setGraph(FactorGraphInterfacePtr graph_);

    // add closures (once, set them as not enabled, to avoid optimizing over
    // them)
    void addClosure(FactorBasePtr factor);

    void resetClosureStatus();
    
    // removes a closure from the pool for confirmation
    void removeClosure(FactorBasePtr factor);

    // add constants, these are variables that represent fixed parameters
    // and are not "touched" by the internal adjustments done while optimizing
    // the system
    void addConstant(VariableBase* constant);

    // does one round of updates, assuming the "central" partition
    // is the one you pass as argumen. The argument is intended to represent
    // the current node where the robot is
    // call this after having inserted all closures of the current epoch
    void compute(VariableBase* variable);

    inline int partitionsNum() const {
      return partitions.size();
    }
    struct ClosureStats {
      enum Status { Accepted = 0x0, Rejected = 0x1, Pending = 0x2 };
      int num_times_checked  = 0;
      int num_times_good     = 0;
      int num_rounds_checked = 0;
      Status status          = Pending;
    };
    using ClosureStatsMap    = std::map<FactorBase*, ClosureStats>;
    using ClosureStatsPtrMap = std::map<FactorBasePtr, ClosureStats*>;

    // here you get the result of the closures, for each open one
    // the graph is not messed up
    const ClosureStatsMap& closures() const {
      return _closures;
    }
    const ClosureStatsPtrMap& updatedClosures() const {
      assert(0 && "dc");
      return _updated_closures;
    }

  protected:
    std::map<FactorBase*, FactorBasePtr> _ptr_sptr_map;
    std::set<VariableBase*> _constants;
    ClosureStatsMap _closures;
    ClosureStatsPtrMap _updated_closures;
    FactorGraphInterfacePtr _graph = 0;
    using FactorGraphViewPtr       = std::unique_ptr<FactorGraphView>;
    std::map<FactorGraphView*, FactorGraphViewPtr> partitions;
    std::map<VariableBase*, FactorGraphView*> var_to_partition_map;
    std::set<VariableBase*> open_variables;

    bool isConstant(VariableBase* var) const;
    void preparePartitioning(VariableBase* active_variable);

    // returns the set where the addition occurred
    // in case of joining
    FactorGraphView* addVariable(FactorGraphView* partition,
                                 VariableBase* variable);

    FactorGraphView* partition(VariableBase* variable);

    void expandPartition(VariableBase* variable);

    void joinPartitions(FactorGraphView* dest, FactorGraphView* src);

    float computeTraversalCost(VariableBase* v1_,
                               VariableBase* v2_);
    void computePartitions();
    void clusterClosures(VariableBase* variable);
    void doValidate1(FactorGraphView* active_partition,
                     std::list<FactorBase*>& factors);
    void doValidate2(FactorGraphView* active_partition,
                     FactorGraphView* other_partition,
                     std::list<FactorBase*>& factors);
    bool doFullfill(VariableBase* fixed_var_,
                    FactorBase* factor_,
                    FactorGraphView* partition);
    bool doFullfillSE2(VariableBase* fixed_var_,
                       FactorBase* factor_,
                       FactorGraphView* partition);
    bool doFullfillSE3(VariableBase* fixed_var_,
                       FactorBase* factor_,
                       FactorGraphView* partition);
    void pushAll(FactorGraphView* partition);
    void popAll(FactorGraphView* partition);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using FactorGraphClosureValidatorPtr =
    std::shared_ptr<FactorGraphClosureValidator>;
} // namespace srrg2_solver
