#include <iomanip>
#include <thread>

// ia system utils
#include <srrg_config/configurable_manager.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>

// ia include linear solver stuff
#include <srrg_solver/solver_core/internals/linear_solvers/instances.h>
// ia include solver stuff (instances)
#include "srrg_solver/solver_core/iteration_algorithm_gn.h"
#include "srrg_solver/solver_core/iteration_algorithm_lm.h"
#include <srrg_solver/solver_core/factor_graph.h>
#include <srrg_solver/solver_core/solver.h>

// ia instances
#include <srrg_solver/solver_core/instances.h>

#include <srrg_solver/variables_and_factors/types_2d/instances.h>
#include <srrg_solver/variables_and_factors/types_3d/instances.h>
#include <srrg_solver/variables_and_factors/types_calib/instances.h>

// ia viewport
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>

// ia drawing action
#include "solver_action_draw.h"

const std::string exe_name = "srrg_solver_gui_app";
#define LOG std::cerr << exe_name + "|"

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace srrg2_solver_gui;

void registerAllTypes() {
  linear_solver_registerTypes();
  solver_registerTypes();
  registerTypes3D();
  registerTypes3D();
  calib_registerTypes();
}

void generateConfig(const std::string& config_file, const std::string& config_name);

void doOptimization(FactorGraphPtr factor_graph_,
                    SolverPtr sparse_solver_,
                    ViewerCanvasPtr canvas_,
                    std::string stats_filename_,
                    std::string output_filename_);

int main(int argc, char** argv) {
  registerAllTypes();

  ParseCommandLine cmd_line(argv);
  ArgumentFlag gen_config(&cmd_line, "j", "generate-config", "generates a config file and quits");
  ArgumentString stats_file(
    &cmd_line, "s", "stats-file", "file where to save the solver statistics", "");
  ArgumentString config_file(
    &cmd_line, "c", "config-file", "config file to read/write", exe_name + ".json");
  ArgumentString config_name(
    &cmd_line, "n", "config-name", "name of the config to load", "my_solver");
  ArgumentString output_file(&cmd_line, "o", "output-file", "file where to save the output", "");
  cmd_line.parse();

  if (gen_config.isSet()) {
    generateConfig(config_file.value(), config_name.value());
    LOG << "configuration generated in [" << FG_YELLOW(config_file.value()) << "]"
        << " with name [" << FG_YELLOW(config_name.value()) << "]\n";
    return 0;
  }

  // ia create solver from config file
  ConfigurableManager manager;
  manager.read(config_file.value());
  SolverPtr solver = manager.getByName<Solver>(config_name.value());
  if (!solver) {
    LOG << "configuration file not found (run with -j)" << std::endl;
    return -1;
  }

  LOG << "configuration readed from [" << FG_YELLOW(config_file.value()) << "]" << std::endl;

  if (cmd_line.lastParsedArgs().empty()) {
    LOG << "no input file, aborting" << std::endl;
    return -1;
  }

  // ia load graph
  const std::string& input_file = cmd_line.lastParsedArgs()[0];
  FactorGraphPtr graph          = FactorGraph::read(input_file);
  if (!graph) {
    LOG << "no graph found in file, [" << FG_RED(input_file) << "], aborting" << std::endl;
    return -1;
  }

  LOG << "loaded [" << FG_YELLOW(input_file) << "] ----- "
      << "variables [" << FG_YELLOW(graph->variables().size()) << "] | "
      << "factors [" << FG_YELLOW(graph->factors().size()) << "]" << std::endl;

  // ia start processing thread
  QApplication qapp(argc, argv);
  srrg2_qgl_viewport::ViewerCoreSharedQGL viewer_core(argc, argv, &qapp, BUFFER_SIZE_100MEGABYTE);
  const ViewerCanvasPtr& canvas = viewer_core.getCanvas(exe_name);

  std::thread processing_t(
    doOptimization, graph, solver, canvas, stats_file.value(), output_file.value());
  viewer_core.startViewerServer();

  processing_t.join();
  return 0;
}

// ia aux functions
void generateConfig(const std::string& config_file, const std::string& config_name) {
  ConfigurableManager manager;
  auto solver = manager.create<Solver>(config_name);

  auto iteration_algorithm = manager.create<IterationAlgorithmLM>();
  solver->param_algorithm.setValue(iteration_algorithm);

  // now we can configure the parameters
  solver->param_max_iterations.value().push_back(100);

  // we set a simplistic termination criteria
  auto term_crit = manager.create<PerturbationNormTerminationCriteria>();
  solver->param_termination_criteria.setValue(term_crit);

  // we install a policy that matches each SE3PosePoseGeodesicQuaternionErrorFactor
  // with a caucy robustifier
  auto bearing_policy = manager.create<RobustifierPolicyByType>();
  bearing_policy->param_factor_class_name.setValue("SE3PosePoseGeodesicQuaternionErrorFactor");

  auto robustifier = manager.create<RobustifierCauchy>();
  bearing_policy->param_robustifier.setValue(robustifier);

  solver->param_robustifier_policies.pushBack(RobustifierPolicyBasePtr(bearing_policy));

  // we now save the configuration
  manager.write(config_file);
}

void doOptimization(FactorGraphPtr factor_graph_,
                    SolverPtr sparse_solver_,
                    ViewerCanvasPtr canvas_,
                    std::string stats_filename_,
                    std::string output_filename_) {
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  factor_graph_->bindFactors();

  // tell the solver which graph to optimize
  sparse_solver_->setGraph(factor_graph_);

  // ia install a preiteration action that flushes the canvas
  sparse_solver_->installPreiterationAction(
    SolverActionDrawPtr(new SolverActionDraw(sparse_solver_.get(), canvas_, factor_graph_)));
  // ia install a postiteration action that flushes the canvas
  sparse_solver_->installPostiterationAction(
    SolverActionDrawPtr(new SolverActionDraw(sparse_solver_.get(), canvas_, factor_graph_)));

  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  size_t sleep_s = 5;
  LOG << "optimization will start in [" << FG_YELLOW(sleep_s) << "] seconds\n";

  while (srrg2_qgl_viewport::ViewerCoreSharedQGL::isRunning()) {
    // ia draw something on the canvas
    for (const auto& v_tuple : factor_graph_->variables()) {
      v_tuple.second->draw(canvas_);
    }

    for (const auto& f : factor_graph_->factors()) {
      f.second->draw(canvas_);
    }

    canvas_->flush();

    if (sleep_s--) {
      std::flush(std::cerr);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      continue;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    if (sparse_solver_->status() == SolverBase::SolverStatus::Error) {
      srrg2_qgl_viewport::ViewerCoreSharedQGL::stop();
      break;
    }

    if (sparse_solver_->status() == SolverBase::SolverStatus::Success) {
      continue;
    }
    // we compute the solution
    LOG << "\ncompute...\n";
    sparse_solver_->compute();
    LOG << "done - perfomed [" << sparse_solver_->currentIteration() << "] iterations";
  }

  LOG << "termination request, printing some stuff\n";

  // at the end the solver has updated the statistics
  LOG << "optimization stats" << std::endl;
  std::cerr << sparse_solver_->iterationStats() << std::endl;

  // write stats
  if (stats_filename_ != "") {
    std::ofstream stream(stats_filename_);
    for (const auto& s : sparse_solver_->iterationStats()) {
      stream << s << std::endl;
    }
    LOG << "statistics written in [" << FG_YELLOW(stats_filename_) << "]" << std::endl;
  }

  // write optimized graph
  if (output_filename_ != "") {
    factor_graph_->write(output_filename_);
    LOG << "output graph written in [" << FG_YELLOW(output_filename_) << "]" << std::endl;
  }
}
