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

  using FactorRawPtrVector      = std::vector<FactorBase*>;
  using LevelFactorPtrVectorMap = std::map<size_t, FactorRawPtrVector>;
  /*! @brief Non linear solver acting on factor graphs */
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
    /*! @return The factor graph interface shared pointer */
    FactorGraphInterfacePtr graph() {
      return _graph;
    }
    /*! @param[in] graph_ factor graph interface shared pointer */
    void setGraph(FactorGraphInterfacePtr graph_) {
      _graph                        = graph_;
      this->_compute_changed_flag   = true;
      this->_structure_changed_flag = true;
    }

    /*! @return Variables modified during the last optimization */
    inline const std::vector<VariableBase*>& activeVariables() const {
      return _active_variables;
    }

    /*! @return Factors which were active during the last optimization */
    inline const LevelFactorPtrVectorMap& activeFactorsPerLevel() const {
      return _active_factors;
    }

    /*! @return final chi square of the last optimization */
    inline float chi2() const {
      return lastIterationStats().chi_normalized;
    }

    void allocateStructures() override;

    void prepareForCompute() override;

    void printAllocation() const;

    /*! Compute the marginal covariance/cross-correlation of a subset of variables
      @param[out] covariance_matrices_ covariance/cross-correlation blocks
      @param[in] variables_ vector of variables pair, if a pair contain the same variable the
      marginal covariance is computed
      @return false on failure
    */
    bool computeMarginalCovariance(MatrixBlockVector& covariance_matrices_,
                                   const VariablePairVector& variables_);
    /*! Extract blocks of the fisher information matrix (approximate hessian) for a subset of
      variables
      @param[out] information_matrices_ fisher information blocks
      @param[in] variables_ vector of variables pair, for each pair the corresponding block of the
      approximate hessian is returned
    */
    void extractFisherInformationBlocks(MatrixBlockVector& covariance_matrices_,
                                        const VariablePairVector& variables_);

  protected:
    /*! Assign to each factor type the corresponding robustifier, following the robustifier_policy
     */
    void assignRobustifiers();
    void prepareForNewLevel() override;
    /*! Connect the variables to the corresponding factors in the graph
      @return false if something goes wrong
     */
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

    /*! Determine which factors partecipate to the optimization */
    void computeActiveFactors();
    /*! Determine the active variables in the current level of the optimization */
    void computeActiveVariables();
    /*! Compute variable ordering */
    void computeOrdering();
    /*! Allocate approximate hessian and gradient vector blocks */
    void allocateWorkspace();

    SparseBlockMatrix _H; /*!< Approximate hessian matrix */
    SparseBlockMatrix _b; /*!< Gradient vector */

    std::vector<VariableBase*> _active_variables; /*!< Container of variables that partecipate to
                                                     the optimizaion in the current level*/
    std::map<size_t, std::vector<FactorBase*>>
      _active_factors; /*!< Container of factors that partecipate to the optimizaion in each level*/
    std::vector<int> _variable_layout; /*!< Block dimension for each variable */
    std::vector<IntPair> _initial_block_layout;
    FactorGraphInterfacePtr _graph = nullptr;
  };

  using SolverPtr = std::shared_ptr<Solver>; /*!<Shared pointer to Solver*/
} // namespace srrg2_solver
