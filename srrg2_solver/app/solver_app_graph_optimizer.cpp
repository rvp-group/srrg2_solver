#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <fstream>
#include <iomanip>

#include <srrg_config/configurable_manager.h>
#include <srrg_system_utils/parse_command_line.h>

#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/internals/linear_solvers/instances.h"
#include "srrg_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholesky_csparse.h"
#include "srrg_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholmod_full.h"
#include "srrg_solver/solver_core/iteration_algorithm_lm.h"
#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/solver_core/termination_criteria.h"
#include "srrg_solver/variables_and_factors/types_2d/instances.h"
#include "srrg_solver/variables_and_factors/types_3d/instances.h"

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace std;

// ia checks whether file exists
inline bool checkFile(const std::string& name_) {
  struct stat buffer;
  return (stat(name_.c_str(), &buffer) == 0);
}

// ia register types
void initTypes() {
  registerTypes2D();
  registerTypes3D();
  solver_registerTypes();
  linear_solver_registerTypes();
}

// ia talking action
class SolverVerboseAction : public SolverActionBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SolverVerboseAction(SolverBase* solver_) : SolverActionBase(solver_) {
  }

  void doAction() final {
    std::cerr << _solver_ptr->lastIterationStats() << std::endl;
  }
};

using SolverVerboseActionPtr = std::shared_ptr<SolverVerboseAction>;

const std::string exe_name = "solver_app_graph_optimizer";
#define LOG std::cerr << exe_name << "|"

// ia generate a default config
void generateConfig(const std::string& config_file_, const std::string& solver_name_);

// ia THE PROGRAM
int main(int argc, char** argv) {
  initTypes();

  ParseCommandLine cmd_line(argv);
  ArgumentFlag param_verbose(&cmd_line,
                             "v",
                             "verbose",
                             "verbose optimization (tells you things while they are happening)",
                             false);
  ArgumentString param_input_file(
    &cmd_line, "i", "input-file", "file containing graph to optimize (*.boss)", "");
  ArgumentFlag param_gen_config(
    &cmd_line, "j", "generate-config", "generates a config file and quits");
  ArgumentString param_stats_file(
    &cmd_line, "s", "stats-file", "file where to save the solver statistics", "");
  ArgumentString param_config_file(
    &cmd_line, "c", "config-file", "config file to read/write", "solver.config");
  ArgumentString param_solver_name(
    &cmd_line,
    "sn",
    "solver-name",
    "name of the solver in the config file (the one to instanciate and run)",
    "solver");
  ArgumentString output_file(&cmd_line, "o", "output-file", "file where to save the output", "");
  cmd_line.parse();

  // ia generate config and exit
  if (param_gen_config.isSet()) {
    generateConfig(param_config_file.value(), param_solver_name.value());
    LOG << "exit\n";
    return 0;
  }

  // ia check if input is provided
  if (!param_input_file.isSet()) {
    std::cerr << cmd_line.options() << std::endl;
    throw std::runtime_error(exe_name + "|ERROR, no input file specified");
  }

  // ia check if config file exists
  if (!checkFile(param_config_file.value())) {
    throw std::runtime_error(exe_name + "|ERROR, cannot find configuration [ " +
                             param_config_file.value() + " ]");
  }

  // ia configurable manager to read config
  LOG << "loading configuration [ " << param_config_file.value() << " ]\n";
  ConfigurableManager manager;
  manager.read(param_config_file.value());

  // ia check if solver with this specific name exists
  LOG << "loading solver [ " << param_config_file.value() << " ]\n";
  SolverPtr solver = manager.getByName<Solver>(param_solver_name.value());
  if (!solver) {
    throw std::runtime_error(exe_name + "|ERROR, cannot find solver with name [ " +
                             param_solver_name.value() + " ] in configuration file [ " +
                             param_config_file.value() + " ]");
  }

  // ia loading graph
  const std::string& input_file = param_input_file.value();
  FactorGraphPtr graph          = FactorGraph::read(input_file);
  if (!graph) {
    throw std::runtime_error(exe_name + "|ERROR, invalid graph file [ " + input_file + " ]");
  }

  // ia talk
  LOG << "loaded [ " << FG_YELLOW(input_file) << " ] ----- "
      << "variables [ " << FG_YELLOW(graph->variables().size()) << " ] | "
      << "factors [ " << FG_YELLOW(graph->factors().size()) << " ]" << std::endl;

  // ia extra step (not required because it's automagically done by the solver)
  // ia binding means creating "links" between variables and factors based on their ids
  graph->bindFactors();

  // ia if we want to be verbose, then we install a talking action
  if (param_verbose.isSet()) {
    SolverVerboseActionPtr talkin_action(new SolverVerboseAction(solver.get()));
    solver->installPostiterationAction(talkin_action);
  }

  // ia provide a graph to the solver and run it
  LOG << FG_YELLOW("BRACE YOURSELF! running solver") << std::endl;
  solver->setGraph(graph);
  solver->compute();

#ifndef NDEBUG
  // ia just in case
  solver->printAllocation();
#endif

  // ia check optimization status
  if (solver->status() == SolverBase::SolverStatus::Success) {
    LOG << FG_GREEN("optimization succeeded :)") << std::endl;
  } else if (solver->status() == SolverBase::SolverStatus::Error) {
    LOG << FG_YELLOW("optimization failed :(") << std::endl;
  }

  // ia write stats
  if (param_stats_file.isSet() && !param_stats_file.value().empty()) {
    LOG << "saving stats in [ " << FG_YELLOW(param_stats_file.value()) << " ]" << std::endl;
    std::ofstream stream(param_stats_file.value());
    for (const auto& s : solver->iterationStats()) {
      stream << s << std::endl;
    }
    // ia good old manners
    stream.close();
  }

  // ia write optimized graph
  if (output_file.isSet() && !output_file.value().empty()) {
    LOG << "saving output in [ " << FG_YELLOW(output_file.value()) << " ]" << std::endl;
    graph->write(output_file.value());
  }

  LOG << "bye man!\n";
  return 0;
}

// --------------------------------------------------------------------------------------------- //
// --------------------------------------------------------------------------------------------- //
// --------------------------------------------------------------------------------------------- //
void generateConfig(const std::string& config_file_, const std::string& solver_name_) {
  ConfigurableManager manager;

  // ia create backup linear solver (default is cholmod)
  auto linear_solver_csparse =
    manager.create<SparseBlockLinearSolverCholeskyCSparse>("linear_solver_csparse");

  // ia create default iteration algorithm (default is GN)
  auto algorithm_gn = manager.create<IterationAlgorithmGN>("gauss-newton");
  algorithm_gn->param_damping.setValue(0.f);

  // ia create backup iteration algorithm (backup is LM)
  auto algorithm_lm = manager.create<IterationAlgorithmLM>("levemberg");
  algorithm_lm->param_lm_iterations_max.setValue(10);

  // ia create default term criteria that stops if the delta_chi < thres (NOT used by default)
  auto term_criteria = manager.create<SimpleTerminationCriteria>("termination_criterium");
  term_criteria->param_epsilon.setValue(1e-3);

  // ia create default robustifier for 2D Pose-Bearing edges
  auto bearing_robustifier = manager.create<RobustifierCauchy>();
  bearing_robustifier->param_chi_threshold.setValue(1.0);

  // ia create robustifier policy for 2D Pose-Bearing edges
  auto bearing_policy = manager.create<RobustifierPolicyByType>();
  bearing_policy->param_factor_class_name.setValue("SE2PosePointBearingErrorFactor");
  bearing_policy->param_robustifier.setValue(bearing_robustifier);

  // ia finally create the solver and setup all the parameters
  auto solver = manager.create<Solver>(solver_name_);
  solver->param_linear_solver.setName("linear_solver_cholmod");
  solver->param_max_iterations.value().push_back(10);
  solver->param_algorithm.setValue(algorithm_gn);
  solver->param_robustifier_policies.pushBack(RobustifierPolicyBasePtr(bearing_policy));
  solver->param_termination_criteria.setValue(nullptr);

  // ia DONE, save and exit
  manager.write(config_file_);
  LOG << "created default (hackable) configuration in [ " << config_file_ << " ]\n";
}
