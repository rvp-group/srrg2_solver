#pragma once
#include <srrg_property/property_vector.h>

#include "factor_base.h"
#include "iteration_algorithm_gn.h"
#include "robustifier.h"
#include "solver_action_base.h"
#include "termination_criteria.h"

namespace srrg2_solver {

  using namespace srrg2_core;

  /*! @brief Generic interface for a solver. A solver is characterized by three parameters :

    - max_iterations to perform in the optimization
    - termination_criteria
    - algorithm which is the optimization algorithm to use (GN, LM ecc)

    To define your solver you must override :
    - buildQuadraticForm() which define how to construct the linear system
     at the current linearization point
    - solveQuadraticForm() how to solve the linear system
    - applyPerturbation() how to use the solution to update the variables involved in the problem
    - push()/pop()/discardTop() how to act on the stack of variables (see VariableBase)
    - getDiagonal() get diagonal of system matrix
    - setDiagonal() set diagonal of system matrix
    - getRHS() getter for target vector
    - getPerturbation() getter for solution of the linear system
    - updateChi() evaluate the squared error at the current iteration

    Notice that at this stage SolverBase knows nothing about factor graphs
  */
  class SolverBase : public Configurable {
    friend class IterationAlgorithmBase;
    friend class TerminationCriteria;

  public:
    /*! Solver status, used to understand whether the optimization was successful or not*/
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

    /*! @return Statistics of the last optimization*/
    inline const IterationStatsVector& iterationStats() const {
      return _iteration_stats;
    }
    /*! Clear stats */
    inline void clearIterationStats() {
      _current_iteration = 0;
      _iteration_stats.clear();
    }
    /*! @return Statistics of the measurements */
    inline const FactorStatsVector& measurementStats(const int& level_ = 0) const {
      auto it = _factor_stats.find(level_);
      return it->second;
    }
    /*! @return Statistics of the last iteration in the optimization */
    inline const IterationStats& lastIterationStats() const {
      return *_iteration_stats.rbegin();
    }
    /*! @return current iteration */
    inline const int& currentIteration() const {
      return _current_iteration;
    }

    inline const int status() const {
      return _status;
    }

    /*! Install a pre-iteration action*/
    void installPreiterationAction(const SolverActionBasePtr& action_);

    /*! Install a post-iteration action*/
    void installPostiterationAction(const SolverActionBasePtr& action_);

    /*! Allocate matrix block structure */
    virtual void allocateStructures();

    /*! Perform all the operations required to compute, can be overrided if necessary */
    virtual void prepareForCompute();

    /*! Solve the optimization problem */
    virtual void compute();

  protected:
    /*! Perform all the operation required to operate on a new level in the hierarchical
     * optimization */
    virtual void prepareForNewLevel() {
      allocateStructures();
    }

    void bindConfigProperties();

    /*! Evaluates the chi square in the current iteration
     @param[out] istats iteration statistics
     */
    virtual bool updateChi(IterationStats& istats) = 0;

    /*! Performs only the linearization of the system
     @param[out] istats iteration statistics
     */
    virtual bool buildQuadraticForm(IterationStats& istats) = 0;

    /*! Performs the solution of the linear system
     @param[out] istats iteration statistics
     */
    virtual bool solveQuadraticForm(IterationStats& istats) = 0;

    /*! Applies the current update to the solution
     @param[out] istats iteration statistics
     */
    virtual void applyPerturbation(IterationStats& istats) = 0;

    /*! @param[out] diagonal of the system matrix*/
    virtual void getDiagonal(std::vector<float>& diagonal) const = 0;

    /*! Setter for the diagonal
      @param[in] diagonal of the system matrix
    */
    virtual void setDiagonal(const std::vector<float>& diagonal) = 0;

    /*! Getter for the target vector in the linear system
      @param[out] b target vector
    */
    virtual void getRHS(std::vector<float>& b) const = 0;

    /*! Getter for the solution of the linear system
      @param[out] dx solution of the linear system
    */
    virtual void getPerturbation(std::vector<float>& dx) const = 0;

    /*! Pushes the current estimates values in the stack*/
    virtual void push() = 0;

    /*! Pop the current estimates values in the stack*/
    virtual void pop() = 0;

    /*! Discards the last stack entry, without touching the estimate */
    virtual void discardTop() = 0;
    /*! Current level of the optimization, used when the problem is hierarchical */
    inline int currentLevel() const {
      return _current_level;
    }

    std::map<int, FactorStatsVector>
      _factor_stats; /*!< Factor stats for each level of the optimization */
    IterationStatsVector _iteration_stats;
    bool _structure_changed_flag = true; /*!< This flag will be set to true if the structure
                                           of linear system has changed */
    bool _compute_changed_flag = true;   /*!< This flag will be set to true if the overall
                                           structure of the problem as changed */
    int _current_level           = 0;
    size_t _max_iterations_total = 0;

    int _current_iteration = 0; /*!< Current iteration of the solver*/

    SolverStatus _status = SolverStatus::Ready; /*!< Status of the solver, exposed to let the
                                                   outside world if everything was ok or not */

    SolverActionBasePtrSet _preiteration_actions; /*!< Pre iteration actions containers. each of
             these actions will be performed BEFORE calling the algorithm oneRound() method */

    SolverActionBasePtrSet _postiteration_actions; /*!< Post iteration actions containers. each of
       these actions will be performed AFTER calling the algorithm oneRound() method */
  };

} // namespace srrg2_solver
