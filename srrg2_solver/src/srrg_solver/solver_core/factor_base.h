#pragma once
#include "solver_stats.h"
#include "srrg_solver/solver_core/internals/sparse_block_matrix/matrix_block.h"
#include "variable_ptr_tuple.h"
#include <srrg_data_structures/correspondence.h>
#include <srrg_data_structures/iterator_interface.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  struct RobustifierBase;

  /*! @brief Base class that represents of factor.
   */
  class FactorBase : public Identifiable,
                     public DrawableBase,
                     protected IteratorInterface_<FactorBase*> {
    friend class FactorGraphInterface;
    friend class FactorBasePtrContainerBase;
    friend class FactorGraph;
    friend class Solver;

    template <typename T>
    friend class IdPtrContainerBase_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*! The id is a 64 bit integer*/
    using Id = int64_t;
    /*! Get the graph id corresponding to this factor */
    inline Id graphId() const {
      return _graph_id;
    }
    /*! Set the graph id for the factor, this is used to identify the factor
     * in the graph */
    inline void setGraphId(Id id_) {
      _graph_id = id_;
    }

    /*! Get factor stats, which include :
      - How many time the measurements is considered an outlier/inliner
      - Chi square, chi square with kernel applied and normalized chi square.
    */
    const FactorStats& stats() const {
      return _stats;
    }

    /*! virtual destructor */
    virtual ~FactorBase(){};

    /*! Set level of hierarchy where the factor is optimized */
    void setLevel(int level_) {
      _current_level = level_;
    }

    int level() const {
      return _current_level;
    }
    /*! Setter for the robustifier */
    inline void setRobustifier(RobustifierBase* r) {
      _robustifier = r;
    }

    /*! Getter for the robustifier */
    inline RobustifierBase* robustifier() {
      return _robustifier;
    }

    /*! Connects the variables with the corresponding factor
     that is for each factor variable, it looks at its id
     and queries the factor container if such a variable is already there
     if not it creates it.
     @param [in] container an associative container the variable id to a variable pointer
     @return number of variables correctly connected to the factor
    */
    virtual int bind(IdVariablePtrContainer& container) = 0;

    /*! number of variables */
    virtual int numVariables() const = 0;

    /*! Called by solver, it sets the H block for the variable r, c
     that is the place where to add the contribution of Jr.transpose() * Jc
     if is_transposed, the target block is transposed and stores
     Jc*Jr
     @param[in] r row index of the approximate hessian block
     @param[in] c col index of the approximate hessian block
     @param[in] block target block pointer
     @param[in] is_transposed, transpose the contribution
    */
    virtual void setHTargetBlock(int r, int c, MatrixBlockBase* block, bool is_transposed) = 0;

    /*! Called by solver, sets the b block where to write the gradient vector
      Jr*e
    @param[in] r row index of the gradient vector
    @param[in] block target block pointer
    */
    virtual void setRHSTargetBlock(int r, MatrixBlockBase* block) = 0;

    /*! invalidates all assignments made to the target blocks*/
    virtual void clearTargetBlocks() = 0;

    /*! Returns the variable pointer in the internal container at position pos
     @param[in] pos index of the internal container where the variable is stored
     @return variable pointer
    */
    virtual VariableBase* variable(int pos) = 0;

    /*! Returns the variable in the internal container at position pos
     @param[in] pos, index of the internal container where the variable is stored
     @return const variable pointer
    */
    virtual const VariableBase* variable(int pos) const = 0;

    /*! True if this factor has to be considered in the optimization
      (at least one variable is "not fixed") */
    virtual bool isActive() const = 0;

    /*! True if the prediction is considered valid */
    virtual bool isValid() const = 0;

    /*! Enables/disables a factor, a factor which is disabled will not be considered in the
     optimization
     @param[in] enabled_ binary flag
     */
    inline void setEnabled(bool enabled_) {
      _enabled = enabled_;
    }
    inline bool enabled() const {
      return _enabled;
    }

    /*! Returns the id of the variable at position pos
      @param[in] pos index of the internal container where the variable is stored
      @return the variable id in the graph
    */
    virtual VariableBase::Id variableId(int pos) const = 0;

    /*! Sets id of the variable at position pos, in the factor
     calling requires rebinding
     @param[in] pos index of the internal container where the variable is stored
     @param[in] id_ the variable id that have to be assigned to the variable
    */
    virtual void setVariableId(int pos, VariableBase::Id id_) = 0;

    /*! Saves each variable in its own stack */
    virtual void pushVariables() = 0;
    /*! Remove the last state of the variabels in their stacks */
    virtual void popVariables() = 0;

    virtual bool variablesTainted() const = 0;

    void draw(ViewerCanvasPtr canvas_) const override;

    /*! override this in the derived class
        here you should:
        - update the H matrix (adding the contribution)
        - update the b vector (adding the contribution)
        - update statistics (chi, status)
       @param[in] chi_only true just evaluate the chi square for the current value of the
       variables
       @param[in] force true impose computation
    */
    virtual void compute(bool chi_only = false, bool force = false) = 0;

  protected:
    FactorBase* _current_it = 0; /*!< A factor can potentially contain multiple factors
                                      of the same type (the factor is actually the iterator
                                      of a container of factors), in the base case this attribute
                                      is equal to "this". More generally this may be a pointer to
                                      the current factor in the IteratorInterface */

    /*! In the internal container place variable v in position pos
          @param[in] pos position in the internal container
          @param[in] v variable pointer to be assigned
        */
    virtual void setVariable(int pos, VariableBase* v) = 0;

    /*! Getter for id of the factor used for serialization (this is a different id with
      respect to the graph id)
    */
    inline int getId() {
      return Identifiable::getId();
    }

    /*! Setter for id of the factor used for serialization (this is a different id with
      respect to the graph id)
    */
    inline void setId(int id_) {
      Identifiable::setId(id_);
    }

    /*! Iterator Interface, for data driven factors
        sets the iterator to the beginning */
    void setBegin() override {
      _current_it = this;
    }

    /*! True if end of container is reached */
    bool isEnd() override {
      return !_current_it;
    }

    /*! Number of elements to iterate, for FactorBase this is equal to 1 */
    size_t size() override {
      return 1;
    }

    /*! Gets the current element */
    FactorBase*& get() override {
      return _current_it;
    }

    /*! Increments the iterator */
    IteratorInterface_<FactorBase*>& next() override {
      _current_it = 0;
      return *this;
    }

    /*! Get the next element of the iterator interface and increments the pointer
      @param[out] datum pointer to the next element
      @return false if at the end of the container
    */
    bool getNext(FactorBase*& datum) override {
      if (!_current_it)
        return false;
      datum       = _current_it;
      _current_it = 0;
      return true;
    }

    RobustifierBase* _robustifier = 0; /*!< Robust Kernel to be used in the factor */
    /*! Calls the robustifier if any
     @return false if no robustification occurred
    */
    bool robustify();
    Id _graph_id            = -1; /*!< Graph id */
    float _kernel_threshold = 0;  /*!< Kernel threshold, if the chi square is above this value the
                                     measurements is considered an outlier */
    FactorStats _stats;           /*!< Factor Stats */
    Eigen::Vector3f
      _kernel_scales; /*!< - _kernel_scales[0] : value of the kernelized cost function,
                           - _kernel_scales[1] : derivative of the kernel function
evaluated in the current linearization point,
                           - _kernel_scales[2] : second order derivative. */
    int _current_level = 0;
    /*!< Level of the hierarchy at which the factor is considered in the optimization */
    bool _enabled = true;
  };

  /*! Associative container, the key is the graph id while the value is pointer to the corresponding
   * factor */
  using IdFactorPtrContainer = AbstractMapView_<FactorBase::Id, FactorBase const*>;
  using IdFactorPair         = std::pair<FactorBase::Id, FactorBase const*>;
  using FactorBasePtr        = std::shared_ptr<FactorBase>; /*!< Shared pointer to FactorBase */

} // namespace srrg2_solver
