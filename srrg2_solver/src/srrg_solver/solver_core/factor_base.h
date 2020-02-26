#pragma once
#include "solver_stats.h"
#include "srrg_solver/solver_core/internals/sparse_block_matrix/matrix_block.h"
#include <srrg_data_structures/iterator_interface.h>
#include <srrg_data_structures/correspondence.h>
#include "variable_ptr_tuple.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  struct RobustifierBase;

  //! base class that represents a factor depending on multiple variables
  //  specialized
  class FactorBase :
    public Identifiable, public DrawableBase,
    protected IteratorInterface_<FactorBase*>{
    friend class FactorGraphInterface;
    friend class FactorBasePtrContainerBase;
    friend class FactorGraph;
    friend class Solver;

    template <typename T>
    friend class IdPtrContainerBase_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    using Id = int64_t;

    inline Id graphId() const {
      return _graph_id;
    }

    inline void setGraphId(Id id_) {
      _graph_id = id_;
    }

    const FactorStats& stats() const {
      return _stats;
    }

    virtual ~FactorBase() {
    };

    inline int maxLevel() const {
      return _max_level;
    }

    void setCurrentLevel(int level_) {
      assert(level_ < _max_level && " invalid level");
      _current_level = level_;
    }

    int currentLevel() const {
      return _current_level;
    }

    inline void setRobustifier(RobustifierBase* r) {
      _robustifier = r;
    }

    inline RobustifierBase* robustifier() {
      return _robustifier;
    }

    // connects the variables with the corresponding factor
    // that is for each factor variable, it looks at its id
    // and queries the factor container if such a variable is already there
    // if not it creates it.
    virtual int bind(IdVariablePtrContainer& container)=0;

    // number of variables
    virtual int numVariables() const = 0;

    // called by solver, it sets the H block for the variable r, c
    // that is the place where to add the contribution of Jr.transpose() * Jc
    // if is_transposed, the target block is transposed and stores
    // Jc*Jr
    virtual void setHTargetBlock(int r, int c, MatrixBlockBase* block, bool is_transposed) = 0;

    // called by solver, sets where to write the coefficient vector
    // Jr*e
    virtual void setRHSTargetBlock(int r, MatrixBlockBase* block) = 0;

    // invalidates all assignments made to the taeget blocks
    virtual void clearTargetBlocks() = 0;

    // returns the variable at pos
    virtual VariableBase* variable(int pos) = 0;

    virtual const VariableBase* variable(int pos) const = 0;

    // true if this factor is considered in the optimization
    // (at least one variable is "not fixed")
    virtual bool isActive() const = 0;

    // true if computation good
    virtual bool isValid() const = 0;

    // enables/disables a factor  a disabled factor is not optimized
    inline void setEnabled(bool enabled_) {
      _enabled = enabled_;
    }
    inline bool enabled() const {
      return _enabled;
    }

    // returns the id of the variable at position pos
    virtual VariableBase::Id variableId(int pos) const = 0;

    // sets id of the variable at position pos, in the factor
    // calling requires rebinding
    virtual void setVariableId(int pos, VariableBase::Id id_) = 0;

    // saves each variable in its own stack
    virtual void pushVariables() = 0;

    virtual void popVariables() = 0;

    virtual bool variablesTainted() const = 0;

    void draw(ViewerCanvasPtr canvas_) const override;

    /** override this in the base class
        here you should:
        - update the H matrix (adding the contribution)
        - update the b vector (adding the contribution)
        - update statistics (chi, status)
    */
    virtual void compute(bool chi_only = false, bool force = false) = 0;
    
  protected:
    FactorBase* _current_it=0;
    virtual void setVariable(int pos, VariableBase* v) = 0;
    // hide these from identifiable to avoid confusion with graph id
    inline int getId() {
      return Identifiable::getId();
    }
    inline void setId(int id_) {
      Identifiable::setId(id_);
    }

    /*Iterator Interface, for data driven factors*/
    // sets the iterator to the beginning
    void setBegin() override {_current_it=this;}

    // true if end is reached
    bool isEnd() override {return !_current_it;}
    // number of elements to iterate

    size_t size() override {return 1;}

    // gets the current element
    FactorBase*& get() override {return _current_it;}

    // increments the iterator
    IteratorInterface_<FactorBase*>& next() override {
      _current_it=0;
      return *this;
    }
    
    // gets the next and increments the pointer
    // returns false if at the end of the container
    bool getNext(FactorBase*& datum) override {
      if (! _current_it)
        return false;
      datum=_current_it;
      _current_it=0;
      return true;
    }   

    RobustifierBase* _robustifier = 0;
    // calls the robustifier if any
    //! returns false if no robustification occurred
    bool robustify();
    Id _graph_id            = -1;
    float _kernel_threshold = 0;
    FactorStats _stats;
    Eigen::Vector3f _kernel_scales;
    int _current_level = 0;
    int _max_level     = 1;
    bool _enabled = true;
  };

  using IdFactorPtrContainer = AbstractMapView_<FactorBase::Id, FactorBase const*>;
  using IdFactorPair = std::pair<FactorBase::Id, FactorBase const*>;
  using FactorBasePtr = std::shared_ptr<FactorBase>;
            
} // namespace srrg2_solver
