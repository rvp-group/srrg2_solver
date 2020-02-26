#include <gtest/gtest.h>

#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>

#include <srrg_solver/solver_core/instances.h>
#include <srrg_solver/solver_core/solver.h>
#include "srrg_solver_extras/types_3d_ad/se3_prior_euler_error_factor_ad.h"

const std::string exe_name = "test_se3_prior_euler_factor_ad";
#define LOG std::cerr << exe_name + "|"

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace srrg2_solver_extras;
const size_t n_iterations = 10;

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(DUMMY_DATA, SE3PriorEulerErrorFactorAD) {
  using VariableType    = VariableSE3EulerRightAD;
  using VariablePtrType = std::shared_ptr<VariableType>;
  using FactorType      = SE3PriorEulerErrorFactorAD;
  using FactorPtrType   = std::shared_ptr<FactorType>;
  using SolverType      = Solver;
  const Vector3f t      = 10 * Vector3f::Random();
  Isometry3f T;
  T.setIdentity();
  T.translation() = t;
  VariablePtrType pose = VariablePtrType(new VariableType);
  pose->setEstimate(Isometry3f::Identity());
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
  // const auto& final_chi2 = stats.back().chi_inliers;
  // assert that relative error is good
  const auto& estimated_T    = pose->estimate();
  const Isometry3f diff_T    = estimated_T.inverse() * T;
  const Vector6f diff_vector = geometry3d::t2ta(diff_T);

  LOG << "Final estimate \n" << estimated_T.matrix() << std::endl;
  LOG << "Ground truth \n" << T.matrix() << std::endl;
  // LOG << stats << std::endl;
  // assert performed iterations are the effectively n_iterations
  // and final chi square
  ASSERT_EQ(stats.size(), n_iterations);
  // ASSERT_LT(final_chi2, 1e-6);

  ASSERT_LT(diff_vector(0), 1e-5);
  ASSERT_LT(diff_vector(1), 1e-5);
  ASSERT_LT(diff_vector(2), 1e-5);
  ASSERT_LT(diff_vector(3), 1e-5);
  ASSERT_LT(diff_vector(4), 1e-5);
  ASSERT_LT(diff_vector(5), 1e-5);
  
}
