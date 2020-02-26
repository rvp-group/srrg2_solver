#pragma once
#include <srrg_property/property_vector.h>

#include "factor_base.h"
#include "iteration_algorithm_gn.h"
#include "robustifier.h"
#include "solver_action_base.h"
#include "termination_criteria.h"

namespace srrg2_solver {

  using namespace srrg2_core;

  // motha of all solvers
  class SolverBase : public Configurable {
    friend class IterationAlgorithmBase;
    friend class TerminationCriteria;

  public:
    //! @brief solver status, to understand whether the optimization was successful or not
    enum SolverStatus : int { Error = -1, Ready = 0, Processing = 1, Success = 2 };

  public:
    PARAM_VECTOR(PropertyVector_<int>,
                 max_iterations,
                 "maximum iterations if no stopping criteria is set",
                 &this->_compute_changed_flag);

    PARAM(PropertyConfigurable_<TerminationCriteria>,
          termination_criteria,
          "term criteria ptr, if 0 solver will do max iterations",
          TerminationCriteriaPtr(new SimpleTerminationCriteria),
          &this->_compute_changed_flag);

    PARAM(PropertyConfigurable_<IterationAlgorithmBase>,
          algorithm,
          "pointer to the optimization algorithm (GN/LM or others)",
          IterationAlgorithmBasePtr(new IterationAlgorithmGN),
          &this->_compute_changed_flag);

    SolverBase();
    virtual ~SolverBase() {
    }

    //! @brief inline accessors
    inline const IterationStatsVector& iterationStats() const {
      return _iteration_stats;
    }

    inline void clearIterationStats() {
      _current_iteration = 0;
      _iteration_stats.clear();
    }

    inline const FactorStatsVector& measurementStats() const {
      return _factor_stats;
    }

    inline const IterationStats& lastIterationStats() const {
      return *_iteration_stats.rbegin();
    }

    inline const int& currentIteration() const {
      return _current_iteration;
    }

    inline const int status() const {
      return _status;
    }

    //! @brief installs a pre-iteration action
    void installPreiterationAction(const SolverActionBasePtr& action_);

    //! @brief installs a post-iteration action
    void installPostiterationAction(const SolverActionBasePtr& action_);

    // allocates the static structures
    virtual void allocateStructures();

    // performs one shot computation before iterations
    virtual void prepareForCompute();

    //! calls oneRound until a maximum number of iterations is reached
    //! or the termination criteria, if installed
    //! tells to stop
    virtual void compute();

  protected:
    void bindConfigProperties();

    // performs one round of optimization
    // returns false if something wrong happened
    // and the iterations are stopped
    // default implementation calls
    // - buildQuadraticForm
    // - solveQuadraticForm
    // - applyPerturbation (if all went well)

    // this evaluates only the chi2 in the current istats
    virtual bool updateChi(IterationStats& istats) = 0;

    // this performs only the linearization of the system;
    virtual bool buildQuadraticForm(IterationStats& istats) = 0;

    // this performs the solution of the linear system
    virtual bool solveQuadraticForm(IterationStats& istats) = 0;

    // this applies the current update to the solution
    virtual void applyPerturbation(IterationStats& istats) = 0;

    // accessors to compute levenberg/dogleg
    virtual void getDiagonal(std::vector<float>& diagonal) const = 0;

    // accessors to compute levenberg
    virtual void setDiagonal(const std::vector<float>& diagonal) = 0;

    // right hand side
    virtual void getRHS(std::vector<float>& b) const = 0;

    // right perturbation from linear system
    virtual void getPerturbation(std::vector<float>& dx) const = 0;

    // pushes the current estimates
    virtual void push() = 0;

    // pops the current estimates
    virtual void pop() = 0;

    // discards the last stack entry, without touching the estimate
    virtual void discardTop() = 0;

    inline int currentLevel() const {
      return _current_level;
    }

    //! @brief attributes
    FactorStatsVector _factor_stats;
    IterationStatsVector _iteration_stats;
    bool _structure_changed_flag = true;
    bool _compute_changed_flag   = true;
    size_t _current_level        = 0;
    size_t _max_iterations_total = 0;

    //! @brief current iteration of the solver (not of the algorithm)
    int _current_iteration = 0;

    //! @brief status of the solver, exposed to let the outside world if everything was ok or not
    SolverStatus _status = SolverStatus::Ready;

    //! @brief pre iteration actions containers. each of these actions will be performed BEFORE
    //!        calling the algorithm OneRound method
    SolverActionBasePtrSet _preiteration_actions;

    //! @brief post iteration actions containers. each of these actions will be performed AFTER
    //!        calling the algorithm OneRound method
    SolverActionBasePtrSet _postiteration_actions;
  };

} // namespace srrg2_solver
