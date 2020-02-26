#include <gtest/gtest.h>

#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/variables_and_factors/types_3d/instances.h"

const std::string exe_name = "test_se3_nicp";
#define LOG std::cerr << exe_name + "|"

using namespace srrg2_core;
using namespace srrg2_solver;

const size_t n_meas       = 1000;
const size_t n_iterations = 10;
using VariableType        = VariableSE3QuaternionRight;
using VariablePtrType     = std::shared_ptr<VariableType>;
using FactorType          = SE3Plane2PlaneErrorFactorCorrespondenceDriven;
using FactorPtrType       = std::shared_ptr<FactorType>;
using SolverType          = Solver;

int main(int argc, char** argv) {
  registerTypes3D();
  solver_registerTypes();
  // linear_solver_registerTypes();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(DUMMY_DATA, SE2Plane2PlaneErrorFactor) {
  const Vector3f t = 10 * Vector3f::Random();
  Quaternionf q(0.3,0.3,0.3,0.3);
  q.normalize();

  Isometry3f T;
  T.setIdentity();
  T.linear() = q.toRotationMatrix();
  T.translation() = t;
  const Isometry3f inv_T   = T.inverse();

  PointNormal3fVectorCloud fixed_cloud;
  PointNormal3fVectorCloud moving_cloud;
  CorrespondenceVector correspondences;
  fixed_cloud.reserve(n_meas);
  moving_cloud.reserve(n_meas);
  correspondences.reserve(n_meas);
  for (size_t m = 0; m < n_meas; ++m) {
    PointNormal3f fixed_point, moving_point;
    fixed_point.coordinates().setRandom();
    fixed_point.normal().setRandom();
    fixed_point.normal().normalize();
    moving_point = fixed_point.transform<TRANSFORM_CLASS::Isometry>(inv_T);
    fixed_cloud.emplace_back(fixed_point);
    moving_cloud.emplace_back(moving_point);
    correspondences.emplace_back(m, m);
  }
  ASSERT_EQ(correspondences.size(), n_meas);

  SolverType solver;
  solver.param_max_iterations.pushBack(n_iterations);
  solver.param_termination_criteria.setValue(nullptr);
  FactorPtrType factor = FactorPtrType(new FactorType);
  factor->setVariableId(0, 0);
  factor->setFixed(fixed_cloud);
  factor->setMoving(moving_cloud);
  factor->setCorrespondences(correspondences);

  FactorGraphPtr graph(new FactorGraph);
  graph->addFactor(factor);
  VariablePtrType pose = VariablePtrType(new VariableType);
  pose->setGraphId(0);
  pose->setEstimate(Isometry3f::Identity());
  graph->addVariable(pose);
  // ia set initial guess and compute
  solver.setGraph(graph);
  solver.compute();
  const auto& stats      = solver.iterationStats();
  const auto& final_chi2 = stats.back().chi_inliers;

  // ia assert performed iterations are the effectively n_iterations
  ASSERT_EQ(stats.size(), n_iterations);
  // ia assert chi2 is good
  ASSERT_LT(final_chi2, 1e-6);
  // ia assert that relative error is good
  const auto& estimated_T    = pose->estimate();
  const Isometry3f diff_T    = estimated_T.inverse() * T;
  const Vector6f diff_vector = geometry3d::t2v(diff_T);

  ASSERT_LT(diff_vector.x(), 1e-5);
  ASSERT_LT(diff_vector.y(), 1e-5);
  ASSERT_LT(diff_vector.z(), 1e-5);
  LOG << stats << std::endl;
}
