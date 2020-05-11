#include <gtest/gtest.h>

#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>

#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/variables_and_factors/types_3d/instances.h"

const std::string exe_name = "test_se3_prior_factor";
#define LOG std::cerr << exe_name + "|"

using namespace srrg2_core;
using namespace srrg2_solver;
const size_t n_iterations = 10;

int main(int argc, char** argv) {
  variables_and_factors_3d_registerTypes();
  solver_registerTypes();
  // linear_solver_registerTypes();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(DUMMY_DATA, SE3PriorErrorFactorAD) {
  using VariableType    = VariableSE3QuaternionRightAD;
  using VariablePtrType = std::shared_ptr<VariableType>;
  using FactorType      = SE3PriorErrorFactorAD;
  using FactorPtrType   = std::shared_ptr<FactorType>;
  using SolverType      = Solver;
  const Vector3f t      = 10 * Vector3f::Random();
  Quaternionf q(0.9, 0.1f, 0.2f, 0.2f);
  q.normalize();

  Isometry3f T;
  T.setIdentity();
  T.linear()      = q.toRotationMatrix();
  T.translation() = t;

  VariablePtrType pose = VariablePtrType(new VariableType);
  pose->setEstimate(Isometry3f::Identity());
  pose->setGraphId(0);
  LOG << "Initial guess \n" << pose->estimate().matrix() << std::endl;
  // assert that relative error is good

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
  const auto& estimated_T    = pose->estimate();
  const Isometry3f diff_T    = estimated_T.inverse() * T;
  const Vector6f diff_vector = geometry3d::t2v(diff_T);
  LOG << "Final estimate \n" << estimated_T.matrix() << std::endl;
  LOG << "Ground truth \n" << T.matrix() << std::endl;

  const auto& stats      = solver.iterationStats();
  const auto& final_chi2 = stats.back().chi_inliers;
  // assert performed iterations are the effectively n_iterations
  // and final chi square
  ASSERT_EQ(stats.size(), n_iterations);
  ASSERT_LT(final_chi2, 1e-6);

  ASSERT_LT(diff_vector(0), 1e-5);
  ASSERT_LT(diff_vector(1), 1e-5);
  ASSERT_LT(diff_vector(2), 1e-5);
  ASSERT_LT(diff_vector(3), 1e-5);
  ASSERT_LT(diff_vector(4), 1e-5);
  ASSERT_LT(diff_vector(5), 1e-5);
}

TEST(DUMMY_DATA, SE3PriorOffsetErrorFactorAD) {
  using VariableType    = VariableSE3QuaternionRightAD;
  using VariablePtrType = std::shared_ptr<VariableType>;
  using FactorType      = SE3PriorOffsetErrorFactorAD;
  using FactorPtrType   = std::shared_ptr<FactorType>;
  using SolverType      = Solver;
  const Vector3f t      = Vector3f::Random();
  Quaternionf q(0.9f, 0.1f, 0.2f, 0.2f);
  q.normalize();
  // tg choose an offset
  Vector6f v        = Vector6f::Random();
  Isometry3f offset = geometry3d::v2t(v);

  Isometry3f T;
  T.setIdentity();
  T.linear()           = q.toRotationMatrix();
  T.translation()      = t;
  Isometry3f Z         = offset.inverse() * T * offset;
  VariablePtrType pose = VariablePtrType(new VariableType);
  pose->setEstimate(Isometry3f::Identity());
  pose->setGraphId(0);
  LOG << "Initial guess \n" << pose->estimate().matrix() << std::endl;
  // assert that relative error is good

  FactorPtrType factor = FactorPtrType(new FactorType);
  factor->setVariableId(0, 0);
  factor->setMeasurement(Z);
  factor->setSensorInRobot(offset);

  FactorGraphPtr graph(new FactorGraph);
  graph->addFactor(factor);
  graph->addVariable(pose);

  SolverType solver;
  solver.param_max_iterations.pushBack(n_iterations);
  solver.param_termination_criteria.setValue(nullptr);
  solver.setGraph(graph);
  solver.compute();
  const auto& estimated_T    = pose->estimate();
  const Isometry3f diff_T    = estimated_T.inverse() * T;
  const Vector6f diff_vector = geometry3d::t2v(diff_T);
  LOG << "Final estimate \n" << estimated_T.matrix() << std::endl;
  LOG << "Ground truth \n" << T.matrix() << std::endl;

  const auto& stats      = solver.iterationStats();
  const auto& final_chi2 = stats.back().chi_inliers;
  // assert performed iterations are the effectively n_iterations
  // and final chi square
  ASSERT_EQ(stats.size(), n_iterations);
  ASSERT_LT(final_chi2, 1e-6);

  ASSERT_LT(diff_vector(0), 1e-5);
  ASSERT_LT(diff_vector(1), 1e-5);
  ASSERT_LT(diff_vector(2), 1e-5);
  ASSERT_LT(diff_vector(3), 1e-5);
  ASSERT_LT(diff_vector(4), 1e-5);
  ASSERT_LT(diff_vector(5), 1e-5);
}
