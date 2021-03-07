#pragma once
#include "factor_base.h"
#include <srrg_data_structures/abstract_ptr_map.h>

namespace srrg2_solver {

  using namespace srrg2_core;

  /*!@brief  General interface that defines the basic functionality to manage a factor graph.
    In the derived class you must specify the accessors to the id to factors and id to variables
    containers.
   */
  class FactorGraphInterface {
  public:
    using VariableFactorMap         = std::multimap<const VariableBase*, FactorBase*>;
    using VariableFactorMapIterator = VariableFactorMap::iterator;
    using Id                        = int64_t;
    /*! Accessor for a variable using the graph id
      @param[in] id the graph id of the variable
      @return The variable pointer
     */
    VariableBase* variable(VariableBase::Id id);
    /*! Accessor for a factor using the graph id
      @param[in] id the graph id of the factor
      @return The factor pointer
     */
    FactorBase* factor(FactorBase::Id id);
    void printVariables();
    virtual ~FactorGraphInterface();
    /*! @return The container of variable pointers  */
    virtual IdVariablePtrContainer& variables() = 0;
    /*! @return The container of factor pointers  */
    virtual IdFactorPtrContainer& factors() = 0;
    /*! Connect a factor with the corresponding variables
      @return Number of variables correctly connected
     */
    virtual int bindFactor(FactorBase* factor);
    /*! Disconnect a factor from the corresponding variables
     */
    virtual void unbindFactor(FactorBase* factor);
    /*! Call bindFactor() for each factor in the interface
      @return Total number of variables connected
    */
    virtual int bindFactors();
    /*! Remove all the factors and variables from the interface */
    virtual void clear();

    /*! Get an iterator to the first element in VariableFactorMap were a variable
      appear, namely the first factor in which the variable is involved.
      @param[in] variable pointer to the variable
      @return Iterator to the corresponding element in _variable_factor_map
    */
    VariableFactorMapIterator lowerFactor(const VariableBase* variable);

    /*! Get an iterator to the element in VariableFactorMap which goes right after a variable,
      namely the first factor in which the variable is NOT involved. Togheter with lowerFactor()
      this function can be used to iterate over all the factors in which a variable is involved.

      @param[in] variable pointer to the variable
      @return Iterator to the corresponding element in _variable_factor_map
    */
    VariableFactorMapIterator upperFactor(const VariableBase* variable);

  protected:
    VariableFactorMap _variable_factor_map; /*!<Associative container
                                              between variables
                                              and factors. Given a variable pointer
                                              you can get all the factors that are connected
                                              to that variable*/

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using FactorGraphInterfacePtr =
    std::shared_ptr<FactorGraphInterface>; /*!< Shared pointer
                                             to FactorGraphInterface */

  /*! @brief View of a factor graph, does not have ownership of variables and factors.
   As such the accessors to the id to variables and id to factors container have a raw pointer
   as value. The compatibility among different container types (same key type different value type)
   is achieved through AbstractMap_, see srrg2_core.
   */
  class FactorGraphView : public FactorGraphInterface {
    friend class FactorGraphClosureValidator;
    friend class FactorGraphViewSelector;
    friend class FactorGraphViewSelector;

  public:
    using IdVariableRawPtrMap = AbstractMap_<VariableBase::Id, VariableBase*>;
    using IdFactorRawPtrMap   = AbstractMap_<FactorBase::Id, FactorBase*>;
    virtual ~FactorGraphView();
    /*! @return The container of variable pointers*/
    IdVariablePtrContainer& variables() override;
    /*! @return The container of factor pointers*/
    IdFactorPtrContainer& factors() override;

  protected:
    IdVariableRawPtrMap _variables; /*!< Id to variable raw pointer container */
    IdFactorRawPtrMap _factors;     /*!< Id to factor raw pointer container */

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  using FactorGraphViewPtr = std::shared_ptr<FactorGraphView>; /*!<Shared pointer
                                                                 to FactorGraphView */

  /*! @brief Materialization of a factor graph. A factor graph *owns* its variables and factors, so
    in this case the id to variable and id to factors container have as value a shared pointer.

    An example of factor graph interface without ownership is FactorGraphView.
  */
  class FactorGraph : public FactorGraphInterface, public Serializable {
  public:
    /*! Add a variable to the factor graph
      @param[in] var a shared pointer to variable
      @return false if the variable was already present in the graph
    */
    bool addVariable(VariableBasePtr var);
    /*! Add a factor to the graph
      @param[in] factor a shared pointer to factor
      @param[in] autobind true if you want to connect the variables on the spot
      @return false if the factor was already present in the graph
    */
    bool addFactor(FactorBasePtr factor, bool autobind = false);
    /*! Remove factor from the graph
      @param[in] factor a shared pointer to factor
      @return false if the factor was not in the graph
    */
    bool removeFactor(FactorBasePtr factor);
    /*! Auxiliary function used to remove a factor from the graph using a raw pointer
      @param[in] factor raw pointer to factor
      @return false if the factor was not present in the graph
    */
    bool removeFactor(FactorBase* factor);

    virtual ~FactorGraph();

    IdVariablePtrContainer& variables() override;
    IdFactorPtrContainer& factors() override;

    void serialize(ObjectData& odata, IdContext& context) override;
    void deserialize(ObjectData& odata, IdContext& context) override;

    /*! Write a factor graph on a file
      @param[in] filename
    */
    void write(const std::string& filename);

    /*! Read a graph from a file (static method)
      @param[in] filename
      @return shared pointer to the factor graph loaded
    */
    static std::shared_ptr<FactorGraph> read(const std::string& filename);
    /*! @return Last graph id*/
    const Id& lastGraphId() const {
      return _last_graph_id;
    }

    inline void setSerializationLevel(const int& level_) {
      _level_serialization = level_;
    }

  protected:
    using IdVariablePtrMap =
      AbstractPtrMap_<VariableBase::Id, VariableBase, std::shared_ptr<VariableBase>>;
    using IdFactorPtrMap = AbstractPtrMap_<FactorBase::Id, FactorBase, std::shared_ptr<FactorBase>>;
    IdFactorPtrMap _factors;     /*!< Id to factor shared pointer container */
    IdVariablePtrMap _variables; /*!< Id to variable shared pointer container */

  private:
    Id _last_graph_id = 0; /*!< Last graph id */
    int _level_serialization = 0;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  using FactorGraphPtr = std::shared_ptr<FactorGraph>; /*!< Shared pointer to FactorGraph */
} // namespace srrg2_solver
