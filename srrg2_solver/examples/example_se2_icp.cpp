#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/internals/linear_solvers/instances.h"
#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/variables_and_factors/types_2d/instances.h"
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>

// Executable name and log
const std::string exe_name = "example_se2_icp";
#define LOG std::cerr << exe_name + "|"

using namespace srrg2_core;
using namespace srrg2_solver;

// Register all the types
void registerAllTypes() {
  variables_and_factors_2d_registerTypes();
  solver_registerTypes();
  linear_solver_registerTypes();
}

int main(int argc, char** argv) {
  registerAllTypes();
  // Parser for the command line to interact with program - see srrg2_core
  ParseCommandLine cmd_line(argv);
  ArgumentInt num_measurements(
    &cmd_line, "n", "num-measurements", "number of measurements to perform alignment", 100);
  ArgumentInt solver_iterations(&cmd_line, "i", "iterations", "number of solver iterations", 20);
  cmd_line.parse();

  // Target transformation matrix
  const Vector3f minimal_T = 10 * Vector3f::Random();
  // v2t map a 3d vector to an element of SE2 - see srrg2_core
  const Isometry2f T     = geometry2d::v2t(minimal_T);
  const Isometry2f inv_T = T.inverse();

  // Read number of measurements from command line
  const size_t& n_meas = num_measurements.value();

  // Instanciate a solver
  Solver solver;

  // Instanciate a factor graph
  FactorGraphPtr graph(new FactorGraph);

  // Create a variable, set a graph id and add it to the graph
  std::shared_ptr<VariableSE2RightAD> pose(new VariableSE2RightAD);
  pose->setGraphId(0);
  graph->addVariable(pose);

  // Create an ICP factor data driven
  std::shared_ptr<SE2Point2PointErrorFactorCorrespondenceDriven> factor(
    new SE2Point2PointErrorFactorCorrespondenceDriven);
  // Set the variable index in the internal container of the factor and the graph id of the
  // variable. This operation has to be repeated for each variable involved in the factor
  factor->setVariableId(0, 0);
  // Add factor to the graph
  graph->addFactor(factor);

  // Create the two cloud that we want to align
  Point2fVectorCloud fixed_cloud;
  Point2fVectorCloud moving_cloud;
  // Vector of associations
  CorrespondenceVector correspondences;

  fixed_cloud.reserve(n_meas);
  moving_cloud.reserve(n_meas);
  correspondences.reserve(n_meas);
  for (size_t i = 0; i < n_meas; ++i) {
    // Create dummy data
    Point2f fixed_point, moving_point;
    fixed_point.coordinates().setRandom();
    moving_point.coordinates() = inv_T * fixed_point.coordinates();
    fixed_cloud.emplace_back(fixed_point);
    moving_cloud.emplace_back(moving_point);
    correspondences.emplace_back(Correspondence(i, i));
  }

  // Setup the correspondence driven factor
  factor->setFixed(fixed_cloud);
  factor->setMoving(moving_cloud);
  factor->setCorrespondences(correspondences);
  factor->setInformationMatrix(Matrix2f::Identity());

  LOG << FG_BBLUE("Non AD correspondence driven factor") << std::endl;
  LOG << "created [" << FG_YELLOW(factor->size()) << "] correspondences\n";

  // Configure the solver - Set max iterations and remove termination criteria (default is
  // SimpleTerminationCriteria)
  solver.param_max_iterations.pushBack(solver_iterations.value());
  solver.param_termination_criteria.setValue(nullptr);
  // Connect the graph to the solver
  solver.setGraph(graph);

  // Set initial guess and compute
  LOG << "compute\n";
  pose->setEstimate(Isometry2f::Identity());
  solver.compute();
  // Visualize statistics and exit
  LOG << "performed [" << FG_YELLOW(solver_iterations.value()) << "] iterations\n";
  const auto& stats = solver.iterationStats();
  LOG << "stats:\n\n" << stats << std::endl;
  LOG << FG_GREEN("exit") << std::endl;
  return 0;
}
