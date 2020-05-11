#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>

#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/internals/linear_solvers/instances.h"

#include "srrg_solver/variables_and_factors/types_2d/instances.h"
#include "srrg_solver/variables_and_factors/types_3d/instances.h"

#include "srrg_solver/solver_core/iteration_algorithm_gn.h"
#include "srrg_solver/solver_core/iteration_algorithm_lm.h"
#include "srrg_solver/solver_core/solver.h"

#include "srrg_solver/utils/g2o_converter/g2o_converter.h"

using namespace srrg2_core;
using namespace srrg2_solver;

// Folder that contains the data for the examples
const std::string example_folder(SRRG2_SOLVER_EXAMPLE_FOLDER);
// Executable name and log
const std::string exe_name = "example_sparse_solver_block_inverse";
#define LOG std::cerr << exe_name + "|"
// Register types
void initTypes() {
  variables_and_factors_2d_registerTypes();
  variables_and_factors_3d_registerTypes();
  solver_registerTypes();
  linear_solver_registerTypes();
}

int main(int argc, char** argv) {
  initTypes();
  // Parser for the command line to interact with the program
  ParseCommandLine cmd(argv);
  // Loading file, if no input specified, load default one
  std::string input_file = "";
  cmd.parse();
  if (cmd.lastParsedArgs().empty()) {
    LOG << "no input file, loading default one" << std::endl;
    input_file = example_folder + "/data/pose_graph_100.g2o";
  } else {
    input_file = cmd.lastParsedArgs()[0];
  }

  std::ifstream stream(input_file);

  // Instanciate a loader that will take care of loading the graph
  G2OConverter g2o_loader;
  g2o_loader.loadGraph(input_file);
  auto graph = g2o_loader.factorGraph();
  // Instanciate the solver
  Solver solver;
  // Configure the solver - Max iterations and termination criteria
  solver.param_max_iterations.value().push_back(10);
  solver.param_termination_criteria.setValue(nullptr);
  // Connect the graph with the solver and compute
  solver.setGraph(graph);
  SystemUsageCounter::tic();
  solver.compute();
  const float time_compute = SystemUsageCounter::toc();
  // Visualize statistics
  LOG << solver.iterationStats() << std::endl;
  // Select variable for which we want to compute the marginal covariance
  auto& V = solver.activeVariables();
  // In this example we choose the first and last variable, we compute
  // the marginal covariance for both plus the cross-correlation between them
  VariablePairVector variables;
  // Take first variable
  const auto& it_begin = V.begin();
  // Create the VariablePair with the variable pointer (Marginal Covariance)
  VariablePair vp = std::make_pair(*it_begin, *it_begin);
  LOG << vp.first->graphId() << " " << vp.second->graphId() << std::endl;
  // Add the pair of variable pointers
  variables.push_back(vp);
  // Take the last variable
  const auto& it_end = V.rbegin();
  // Cross-correlation block
  vp = std::make_pair(*it_begin, *it_end);
  LOG << vp.first->graphId() << " " << vp.second->graphId() << std::endl;
  variables.push_back(vp);
  // Marginal covariance of the last variable
  vp = std::make_pair(*it_end, *it_end);
  LOG << vp.first->graphId() << " " << vp.second->graphId() << std::endl;
  variables.push_back(vp);
  // Container where the marginal covariance/cross-correlation will be added
  // same order as for the VariablePairVector. This blocks can be converted
  // to an Eigen type using the eigenType<EigenType>() method (where EigenType is a template)
  MatrixBlockVector matricies;
  SystemUsageCounter::tic();
  // Compute
  solver.computeMarginalCovariance(matricies, variables);
  const float t = SystemUsageCounter::toc();
  // Print blocks and timing that exit
  for (const auto& block : matricies) {
    block->print();
  }
  LOG << "Time for compute : " << time_compute << std::endl;
  LOG << "Time inverse computation : " << t << std::endl;
  return 0;
}
