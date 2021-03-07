#include <cassert>
#include <srrg_property/property_vector.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_config/configurable_command.h>

#include "iteration_algorithm_base.h"
#include "iteration_algorithm_gn.h"
#include "solver_base.h"
#include "termination_criteria.h"

namespace srrg2_core {
  class ConfigurableShell;
}
// motha of all solvers
namespace srrg2_solver {
  using namespace std;
  using namespace srrg2_core;

  bool SolverBase::cmdCompute(std::string& response) {
    response = className() + "|compute... ";
    compute();
    response += "Done";
    return true;
  }

  bool SolverBase::cmdStats(std::string& response) {
    std::ostringstream os;
    os << className() + "|stats" << std::endl;
    os << iterationStats() << std::endl;
    response = os.str();
    return true;
  }
  
  SolverBase::SolverBase() {
    addCommand (new ConfigurableCommand_
                < SolverBase,
                typeof(&SolverBase::cmdCompute),
                std::string>
                (this,
                 "compute",
                 "starts a computation",
                 &SolverBase::cmdCompute
                 ));
    addCommand (new ConfigurableCommand_
                < SolverBase,
                typeof(&SolverBase::cmdStats),
                std::string>
                (this,
                 "stats",
                 "prints the stats of the last computation",
                 &SolverBase::cmdStats
                 ));
  }
  

  void SolverBase::installPreiterationAction(const SolverActionBasePtr& action_) {
    if (_preiteration_actions.count(action_)) {
      throw std::runtime_error("SolverBase::installPreiterationAction|action already installed");
    }

    _preiteration_actions.insert(action_);
    std::cerr << "SolverBase::installPreiterationAction|installed [" << _preiteration_actions.size()
              << "] pre-iteration actions" << std::endl;
  }

  void SolverBase::installPostiterationAction(const SolverActionBasePtr& action_) {
    if (_postiteration_actions.count(action_)) {
      throw std::runtime_error("SolverBase::installPreiterationAction|action already installed");
    }

    _postiteration_actions.insert(action_);
    std::cerr << "SolverBase::installPostiterationAction|installed ["
              << _postiteration_actions.size() << "] post-iteration actions" << std::endl;
  }

  void SolverBase::allocateStructures() {
    _structure_changed_flag = false;
  }

  void SolverBase::prepareForCompute() {
    _max_iterations_total = 0;
    for (size_t i = 0; i < param_max_iterations.size(); ++i)
      _max_iterations_total += param_max_iterations.value(i);

    if (!_max_iterations_total) {
      std::cerr << FG_YELLOW("SolverBase::prepareForCompute|number of iterations == 0")
                << std::endl;
    }
    _iteration_stats.clear();
    _iteration_stats.reserve(_max_iterations_total);

    if (!param_algorithm.value()) {
      throw std::runtime_error("SolverBase::prepareForCompute|no algorithm set");
    }
    _compute_changed_flag = false;
  }

  void SolverBase::prepareForNewLevel() {
    
    allocateStructures();
  }

  void SolverBase::compute() {
    if (_compute_changed_flag) {
      prepareForCompute();
    }

    param_algorithm->setSolver(this);
    TerminationCriteriaPtr tc = param_termination_criteria.value();
    if (tc) {
      param_termination_criteria->setSolver(this);
    }
    _iteration_stats.clear();
    _current_iteration = 0;
    _status            = SolverBase::SolverStatus::Processing;
    int highest_level  = param_max_iterations.size() - 1;
    // process the levels from highest to lowest
    for (_current_level = highest_level; _current_level >= 0; --_current_level) {
      int max_level_iterations = param_max_iterations.value(_current_level);
      // prevent to cancel the internal structures for no iterations on current level
      if (!max_level_iterations) {
        continue;
      }
      prepareForNewLevel();
      for (int level_iteration = 0; level_iteration < max_level_iterations; ++level_iteration) {
        // ia perform all the pre-iteration actions
        for (SolverActionBasePtr pre_action : _preiteration_actions) {
          pre_action->doAction();
        }
        // ia do the trick
        const bool solution_ok = param_algorithm->oneRound();
        ++_current_iteration;

        // ia perform all the post-iteration actions
        for (SolverActionBasePtr post_action : _postiteration_actions) {
          post_action->doAction();
        }

        // ia check if algorithm failed
        if (!solution_ok) {
          cerr << "SolverBase::compute|solver fail" << endl;
          _status = SolverBase::SolverStatus::Error;
          return;
        }

        // ia check if we have to stop
        if (tc && tc->hasToStop()) {
          break;
        }
      }
    }
    _current_level = 0;
    _status        = SolverBase::SolverStatus::Success;
  }
} // namespace srrg2_solver
