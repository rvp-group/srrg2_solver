#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>

#include "srrg_solver/solver_core/internals/linear_solvers/instances.h"
#include "srrg_solver/solver_core/instances.h"

#include "srrg_solver/variables_and_factors/types_2d/instances.h"
#include "srrg_solver/variables_and_factors/types_3d/instances.h"

#include "srrg_solver/solver_core/iteration_algorithm_gn.h"
#include "srrg_solver/solver_core/iteration_algorithm_lm.h"
#include "srrg_solver/solver_core/solver.h"

#include "srrg_solver/utils/g2o_converter/g2o_converter.h"

using namespace srrg2_core;
using namespace srrg2_solver;

const std::string example_folder(SRRG2_SOLVER_EXAMPLE_FOLDER);
const std::string exe_name = "example_sparse_solver_block_inverse";
#define LOG std::cerr << exe_name + "|"

void initTypes() {
  registerTypes2D();
  registerTypes3D();
  solver_registerTypes();
  linear_solver_registerTypes();
}

int main(int argc, char** argv) {
  initTypes();
  ParseCommandLine cmd(argv);

  // ia loading file, if no input specified, load default one
  std::string input_file = "";
  cmd.parse();
  if (cmd.lastParsedArgs().empty()) {
    LOG << "no input file, loading default one" << std::endl;
    input_file = example_folder + "/data/pose_graph_100.g2o";
  } else {
    input_file = cmd.lastParsedArgs()[0];
  }

  std::ifstream stream(input_file);

  // a loader will take care of loading the graph
  G2OConverter g2o_loader;
  g2o_loader.loadGraph(input_file);
  auto graph = g2o_loader.factorGraph();

  Solver solver;
  solver.param_max_iterations.value().push_back(10);
  solver.setGraph(graph);
  SystemUsageCounter::tic();
  solver.compute();
  const float time_compute = SystemUsageCounter::toc();
  LOG << solver.iterationStats() << std::endl;

  auto& V              = solver.activeVariables();
  const auto& it_begin = V.begin();
  VariablePair vp      = std::make_pair(*it_begin, *it_begin);
  LOG << vp.first->graphId() << " " << vp.second->graphId() << std::endl;
  VariablePairVector variables;
  variables.push_back(vp);
  const auto& it_end = V.rbegin();
  vp                 = std::make_pair(*it_begin, *it_end);
  LOG << vp.first->graphId() << " " << vp.second->graphId() << std::endl;
  variables.push_back(vp);
  vp = std::make_pair(*it_end, *it_begin);
  LOG << vp.first->graphId() << " " << vp.second->graphId() << std::endl;
  variables.push_back(vp);
  MatrixBlockVector matricies;

  SystemUsageCounter::tic();
  solver.computeMarginalCovariance(matricies, variables);
  const float t = SystemUsageCounter::toc();
  for (const auto& block : matricies) {
    block->print();
  }
  LOG << "Time for compute : " << time_compute << std::endl;
  LOG << "Time inverse computation : " << t << std::endl;
  return 0;
}
