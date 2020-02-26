#pragma once
#include "factor_graph.h"
#include "robustifier_policy.h"
#include "solver_base.h"
#include "srrg_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholmod_full.h"
#include <srrg_config/property_configurable_vector.h>

namespace srrg2_solver {

  using namespace srrg2_core;


  using VariablePair       = std::pair<VariableBase*, VariableBase*>;
  using VariablePairVector = std::vector<VariablePair>;
  using MatrixBlockVector  = std::vector<MatrixBlockBase*>;
  // @brief the non linear solver class
  class Solver : public SolverBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PARAM(PropertyConfigurable_<SparseBlockLinearSolver>,
          linear_solver,
          "pointer to linear solver used to compute Hx=b",
          SparseBlockLinearSolverPtr(new SparseBlockLinearSolverCholmodFull),
          &this->_structure_changed_flag);

    PARAM_VECTOR(PropertyConfigurableVector_<RobustifierPolicyBase>,
                 robustifier_policies,
                 "policies used to assign robust kernels",
                 0);

    PARAM(PropertyFloat,
          mse_threshold,
          "Minimum mean square error variation to perform global optimization",
          -1.f,
          0);

    Solver();

    FactorGraphInterfacePtr graph() {
      return _graph;
    }

    void setGraph(FactorGraphInterfacePtr graph_) {
      _graph                        = graph_;
      this->_compute_changed_flag   = true;
      this->_structure_changed_flag = true;
    }

    // variables modified during optimization
    inline const std::vector<VariableBase*>& activeVariables() const {
      return _active_variables;
    }

    // factors used during optimization
    inline const std::vector<FactorBase*>& activeFactors() const {
      return _active_factors;
    }
    
    // getter for the chi square error of the last optimization
    inline float meanSquareError() const {
      return _mse_last_iter;
    }
    
    // validates the setup parameters (call once)
    void allocateStructures() override;

    // performs the one shot initialization
    // allocates structures in linear solver
    // populates active factors and active variables
    virtual void prepareForCompute() override;

    void compute();

    // debug
    void printAllocation() const;

    // get the covariance matrix for a specific variables pair
    bool computeMarginalCovariance(MatrixBlockVector& covariance_matricies_,
                                   const VariablePairVector& variables_);

  protected:
    void assignRobustifiers();
    bool bindFactors();
    bool updateChi(IterationStats& istat) override;
    bool buildQuadraticForm(IterationStats& istat) override;
    bool solveQuadraticForm(IterationStats& istat) override;
    void applyPerturbation(IterationStats& istat) override;
    void getDiagonal(std::vector<float>& diagonal) const override;
    void setDiagonal(const std::vector<float>& diagonal) override;
    void getRHS(std::vector<float>& b) const override;
    void getPerturbation(std::vector<float>& dx) const override;
    void push() override;
    void pop() override;
    void discardTop() override;

    // the ones below are called in init
    void computeActiveRegion();
    void computeOrdering();
    void allocateWorkspace();

    SparseBlockMatrix _H; // in place operation
    SparseBlockMatrix _b; // used also for the update

    // @brief : Mean Square Error after last optimization
    float _mse_last_iter;

    // number of effective active factors (include expansion of data-driven ones)
    int _num_active_factors;
    
    // solver interface
    std::vector<VariableBase*> _active_variables;
    std::vector<FactorBase*> _active_factors;
    std::vector<int> _variable_layout;
    std::vector<IntPair> _initial_block_layout;
    FactorGraphInterfacePtr _graph = 0;
  };

  using SolverPtr = std::shared_ptr<Solver>;
} // namespace srrg2_solver
