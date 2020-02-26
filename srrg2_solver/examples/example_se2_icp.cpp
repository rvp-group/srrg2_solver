#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>

// ia include linear solver stuff
#include "srrg_solver/variables_and_factors/types_2d/instances.h"
#include "srrg_solver/solver_core/internals/linear_solvers/instances.h"
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/solver.h"

const std::string exe_name = "example_se2_icp";
#define LOG std::cerr << exe_name + "|"

using namespace srrg2_core;
using namespace srrg2_solver;

// ia register all the types (REQUIRED)
void registerAllTypes() {
  registerTypes2D();
  solver_registerTypes();
  linear_solver_registerTypes();
}

int main(int argc, char** argv) {
  registerAllTypes();

  ParseCommandLine cmd_line(argv);
  ArgumentInt num_measurements(
    &cmd_line, "n", "num-measurements", "number of measurements to perform alignment", 100);
  ArgumentInt solver_iterations(&cmd_line, "i", "iterations", "number of solver iterations", 20);
  cmd_line.parse();

  // ia this is what we want to estimate
  const Vector3f minimal_T = 10 * Vector3f::Random();
  const Isometry2f T       = geometry2d::v2t(minimal_T);
  const Isometry2f inv_T   = T.inverse();

  // ia generate measurements and relative factors
  const size_t& n_meas = num_measurements.value();

  // declare a _solver
  Solver solver;

  // declare a graph,
  FactorGraphPtr graph(new FactorGraph);

  // create a variable, set an id and add it to the graph
  std::shared_ptr<VariableSE2RightAD> pose(new VariableSE2RightAD);
  pose->setGraphId(0);
  graph->addVariable(pose);

  // create an ICP factor, correspondence driven, set the var index, and add it to the graph
  std::shared_ptr<SE2Point2PointErrorFactorCorrespondenceDriven> factor(
    new SE2Point2PointErrorFactorCorrespondenceDriven);
  factor->setVariableId(0, 0);
  graph->addFactor(factor);

  // ia create two cloud that we want to align, and a vector of correspondences
  Point2fVectorCloud fixed_cloud;
  Point2fVectorCloud moving_cloud;
  CorrespondenceVector correspondences;

  fixed_cloud.reserve(n_meas);
  moving_cloud.reserve(n_meas);
  correspondences.reserve(n_meas);
  for (size_t i = 0; i < n_meas; ++i) {
    // ia create dummy measurements
    Point2f fixed_point, moving_point;
    fixed_point.coordinates().setRandom();
    moving_point.coordinates() = inv_T * fixed_point.coordinates();
    fixed_cloud.emplace_back(fixed_point);
    moving_cloud.emplace_back(moving_point);
    correspondences.emplace_back(Correspondence(i, i));
  }

  // setup the factor;
  factor->setFixed(fixed_cloud);
  factor->setMoving(moving_cloud);
  factor->setCorrespondences(correspondences);
  factor->setInformationMatrix(Matrix2f::Identity());

  LOG << FG_BBLUE("Non AD correspondence driven factor") << std::endl;
  LOG << "created [" << FG_YELLOW(factor->size()) << "] correspondences\n";

  // ia configure max iterations and remove termination criteria to perform all iterations
  solver.param_max_iterations.pushBack(solver_iterations.value());
  solver.param_termination_criteria.setValue(nullptr);
  solver.setGraph(graph);

  // ia set initial guess and compute
  LOG << "compute\n";
  pose->setEstimate(Isometry2f::Identity());
  solver.compute();

  LOG << "performed [" << FG_YELLOW(solver_iterations.value()) << "] iterations\n";
  const auto& stats = solver.iterationStats();
  LOG << "stats:\n\n" << stats << std::endl;
  LOG << FG_GREEN("exit") << std::endl;
}
