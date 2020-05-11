#include <gtest/gtest.h>

#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>

#include "srrg_solver/variables_and_factors/types_2d/instances.h"
#include "srrg_solver/solver_core/internals/linear_solvers/instances.h"
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/solver.h"

const std::string exe_name = "test_se2_prior_factor";
#define LOG std::cerr << exe_name + "|"

using namespace srrg2_core;
using namespace srrg2_solver;
using VariableType    = VariableSE2Right;
using VariablePtrType = std::shared_ptr<VariableType>;
using FactorType      = SE2PriorErrorFactor;
using FactorPtrType   = std::shared_ptr<FactorType>;
using SolverType      = Solver;

const size_t n_iterations = 10;

int main(int argc, char** argv) {
  variables_and_factors_2d_registerTypes();
  solver_registerTypes();
  linear_solver_registerTypes();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(DUMMY_DATA, SE2PriorErrorFactor) {
  Vector3f pose_vector = 10 * Vector3f::Random();
  Isometry2f T         = geometry2d::v2t(pose_vector);
  VariablePtrType pose = VariablePtrType(new VariableType);
  pose->setEstimate(Isometry2f::Identity());
  pose->setGraphId(0);
  LOG << "Initial guess \n" << pose->estimate().matrix() << std::endl;
  
  FactorPtrType factor = FactorPtrType(new FactorType);
  factor->setVariableId(0, 0);
  factor->setMeasurement(T);

  FactorGraphPtr graph(new FactorGraph);
  graph->addFactor(factor);
  graph->addVariable(pose);

  SolverType solver;  
  solver.param_max_iterations.pushBack(n_iterations);
  solver.param_termination_criteria.setValue(nullptr);
  solver.setGraph(graph);
  solver.compute();

  const auto& stats      = solver.iterationStats();
  const auto& final_chi2 = stats.back().chi_inliers;

  // assert performed iterations are the effectively n_iterations
  // and final chi square
  ASSERT_EQ(stats.size(), n_iterations);
  ASSERT_LT(final_chi2, 1e-6);
  // assert that relative error is good
  const auto& estimated_T    = pose->estimate();
  const Isometry2f diff_T    = estimated_T.inverse() * T;
  const Vector3f diff_vector = geometry2d::t2v(diff_T);

  ASSERT_LT(diff_vector.x(), 1e-5);
  ASSERT_LT(diff_vector.y(), 1e-5);
  ASSERT_LT(diff_vector.z(), 1e-5);
  LOG << "Final estimate \n" << estimated_T.matrix() << std::endl;
  LOG << "Ground truth \n" << T.matrix() << std::endl;
}
