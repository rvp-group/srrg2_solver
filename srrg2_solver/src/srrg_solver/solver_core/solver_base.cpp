#include <cassert>
#include <srrg_property/property_vector.h>
#include <srrg_system_utils/shell_colors.h>

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

  struct CommandCompute : public Configurable::CommandBase {
  public:
    CommandCompute(SolverBase* solver) :
      Configurable::CommandBase(solver, "compute", ": runs the optimizer") {
    }

    bool execute(srrg2_core::ConfigurableShell* shell_,
                 std::string& response,
                 const std::vector<std::string>& tokens) override {
      std::ostringstream os;
      os << "module: " << _configurable->className() << " ptr: " << _configurable << std::endl;
      os << "computing ... " << std::endl;
      SolverBase* solver = dynamic_cast<SolverBase*>(_configurable);
      if (!solver) {
        throw std::runtime_error("type mismatch");
      }
      solver->compute();
      os << "Done!" << std::endl;
      response = os.str();
      return true;
    }
  };

  struct CommandStats : public Configurable::CommandBase {
  public:
    CommandStats(SolverBase* solver) :
      Configurable::CommandBase(solver, "stats", ": displays the statistics") {
    }

    bool execute(srrg2_core::ConfigurableShell* shell_,
                 std::string& response,
                 const std::vector<std::string>& tokens) override {
      std::ostringstream os;
      os << "module: " << _configurable->className() << " ptr: " << _configurable << std::endl;
      os << "stats: " << std::endl;
      SolverBase* solver = dynamic_cast<SolverBase*>(_configurable);
      if (!solver) {
        throw std::runtime_error("type mismatch");
      }
      os << solver->iterationStats() << std::endl;
      response = os.str();
      return true;
    }
  };

  SolverBase::SolverBase() {
    addCommand(new CommandStats(this));
    addCommand(new CommandCompute(this));
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

    _iteration_stats.reserve(_max_iterations_total);
    _iteration_stats.clear();

    if (!param_algorithm.value()) {
      throw std::runtime_error("SolverBase::prepareForCompute|no algorithm set");
      // algorithm.setValue(std::shared_ptr<IterationAlgorithmBase>(new IterationAlgorithmGN));
    }
    _compute_changed_flag = false;
  }

  void SolverBase::compute() {
    if (_structure_changed_flag) {
      allocateStructures();
    }

    if (_compute_changed_flag) {
      prepareForCompute();
    }

    param_algorithm->setSolver(this);
    TerminationCriteriaPtr tc = param_termination_criteria.value();
    if (tc) {
      param_termination_criteria->setSolver(this);
    }
    _iteration_stats.clear();
    _status = SolverBase::SolverStatus::Processing;
    for (_current_level = 0; _current_level < param_max_iterations.size(); ++_current_level) {
      int max_level_iterations = param_max_iterations.value(_current_level);
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
          //          std::cerr << "SolverBase::compute|stopped by termination criteria" <<
          //          std::endl;
          break;
        }
      }
    }
    _current_level = 0;
    _status        = SolverBase::SolverStatus::Success;
  }
} // namespace srrg2_solver
