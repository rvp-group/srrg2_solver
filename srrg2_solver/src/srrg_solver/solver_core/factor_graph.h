#pragma once
#include "factor_base.h"
#include <srrg_data_structures/abstract_ptr_map.h>

namespace srrg2_solver {

  using namespace srrg2_core;

  class FactorGraphInterface {
  public:
    using VariableFactorMap         = std::multimap<const VariableBase*, FactorBase*>;
    using VariableFactorMapIterator = VariableFactorMap::iterator;
    using Id                        = int64_t;
    // sugar
    VariableBase* variable(VariableBase::Id id);
    FactorBase* factor(FactorBase::Id id);
    void printVariables() const; // debug
    virtual ~FactorGraphInterface();
    // essence
    virtual const IdVariablePtrContainer& variables() const  = 0;
    virtual IdVariablePtrContainer& variables()              = 0;
    virtual const IdFactorPtrContainer& factors() const = 0;
    virtual IdFactorPtrContainer& factors()             = 0;
    virtual int bindFactor(FactorBase* factor);
    virtual void unbindFactor(FactorBase* factor);
    virtual int bindFactors();
    virtual void clear(); // clears the interface

    // to iterate on all factors of a variable.
    VariableFactorMapIterator lowerFactor(const VariableBase* variable);
    VariableFactorMapIterator upperFactor(const VariableBase* variable);

  protected:
    VariableFactorMap _variable_factor_map;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  using FactorGraphInterfacePtr = std::shared_ptr<FactorGraphInterface>;

  // view of a factor graph, can be created by a friend class
  // does not own the storage
  class FactorGraphView : public FactorGraphInterface {
    friend class FactorGraphClosureValidator;
    friend class FactorGraphViewSelector;
    friend class FactorGraphViewSelector;

  public:
    using IdVariableRawPtrMap = AbstractMap_<VariableBase::Id, VariableBase const*>;
    using IdFactorRawPtrMap   = AbstractMap_<FactorBase::Id, FactorBase const*>;
    virtual ~FactorGraphView();
    virtual const IdVariablePtrContainer& variables() const;
    virtual IdVariablePtrContainer& variables();
    virtual const IdFactorPtrContainer& factors() const;
    virtual IdFactorPtrContainer& factors();

  protected:
    IdVariableRawPtrMap _variables;
    IdFactorRawPtrMap _factors;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  using FactorGraphViewPtr = std::shared_ptr<FactorGraphView>;

  // materialization of a factor graph
  // a factor graph *owns* its members
  class FactorGraph : public FactorGraphInterface, public Serializable {
  public:
    bool addVariable(VariableBasePtr var);
    bool addFactor(FactorBasePtr factor, bool autobind = false);
    bool removeFactor(FactorBasePtr factor);

    bool removeFactor(FactorBase* factor); // tudu: put protected
    // bool removeVariable(VariableBase* var);
    virtual ~FactorGraph();

    const IdVariablePtrContainer& variables() const override;
    IdVariablePtrContainer& variables() override;
    const IdFactorPtrContainer& factors() const override;
    IdFactorPtrContainer& factors() override;

    void serialize(ObjectData& odata, IdContext& context) override;
    void deserialize(ObjectData& odata, IdContext& context) override;

    // convenience function to write a factor graph to a file
    void write(const std::string& filename);

    // convenience functions to read a graph from a file
    static std::shared_ptr<FactorGraph> read(const std::string& filename);

    const Id& lastGraphId() const {
      return _last_graph_id;
    }

  protected:
    using IdVariablePtrMap =
      AbstractPtrMap_<VariableBase::Id, VariableBase, std::shared_ptr<VariableBase>>;
    using IdFactorPtrMap =
      AbstractPtrMap_<FactorBase::Id, FactorBase, std::shared_ptr<FactorBase>>;
    IdFactorPtrMap _factors; // uses the factors to store the measurements
    IdVariablePtrMap _variables;

  private:
    Id _last_graph_id = 0;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  using FactorGraphPtr = std::shared_ptr<FactorGraph>;
} // namespace srrg2_solver
