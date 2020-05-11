#include <thread>
// ia system utils
#include <srrg_config/configurable_manager.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>
// ia solver stuff
#include <srrg_solver/solver_core/instances.h>
#include <srrg_solver/solver_core/internals/linear_solvers/instances.h>
#include <srrg_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholesky_csparse.h>
#include <srrg_solver/solver_core/iteration_algorithm_lm.h>
#include <srrg_solver/solver_core/solver.h>
#include <srrg_solver/utils/factor_graph_utils/instances.h>
#include <srrg_solver/variables_and_factors/types_2d/instances.h>
#include <srrg_solver/variables_and_factors/types_3d/instances.h>
#include <srrg_solver/variables_and_factors/types_calib/instances.h>
// ia viewer stuff
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>

const std::string exe_name = "test_graph_viewer|";
#define LOG std::cerr << exe_name

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace srrg2_qgl_viewport;

void initTypes() {
  variables_and_factors_2d_registerTypes();
  variables_and_factors_3d_registerTypes();
  variables_and_factors_calib_registerTypes();
  solver_registerTypes();
  linear_solver_registerTypes();
  solver_utils_registerTypes();
}

const char* banner[] = {"showing a standard 3d PoseGraph.", "usage: <exe> <filename>", 0};

void viewGraph(FactorGraphPtr graph_, ViewerCanvasPtr canvas_);

int main(int argc, char** argv) {
  initTypes();

  ParseCommandLine cmd_line(argv);
  cmd_line.parse();

  if (cmd_line.lastParsedArgs().empty()) {
    throw std::runtime_error("you forgot the input file, exiting");
  }

  // ia loading graph
  const std::string& input_file = cmd_line.lastParsedArgs()[0];
  LOG << "loading graph from file [" << FG_YELLOW(input_file) << "]" << std::endl;

  FactorGraphPtr graph = FactorGraph::read(input_file);
  if (!graph) {
    throw std::runtime_error("invalid file, exiting");
  }

  LOG << "loaded graph with " << FG_YELLOW(graph->variables().size()) << " variables and "
      << FG_YELLOW(graph->factors().size()) << " factors" << std::endl;

  LOG << "binding factors ... ";
  size_t new_vars = graph->bindFactors();
  std::cerr << "created " << FG_YELLOW(new_vars) << " new variables" << std::endl;

  QApplication qapp(argc, argv);
  ViewerCoreSharedQGL viewer_core(argc, argv, &qapp);

  std::thread graph_t(viewGraph, graph, viewer_core.getCanvas("viewer_core_shared_canvas"));
  viewer_core.startViewerServer();

  graph_t.join();
  return 0;
}

void viewGraph(FactorGraphPtr graph_, ViewerCanvasPtr canvas_) {
  while (ViewerCoreSharedQGL::isRunning()) {
    for (auto v : graph_->variables()) {
      v.second->draw(canvas_);
    }

    for (auto f : graph_->factors()) {
      f.second->draw(canvas_);
    }

    canvas_->flush();
  }
}
