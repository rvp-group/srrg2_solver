#include <gtest/gtest.h>

#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>

// ia include solver stuff (instances)
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/solver.h"
// ia include types stuff (instances)
#include "srrg_solver/variables_and_factors/types_2d/instances.h"

const std::string exe_name = "test_se2_nicp";
#define LOG std::cerr << exe_name + "|"

using namespace srrg2_core;
using namespace srrg2_solver;

const size_t n_meas       = 1000;
const size_t n_iterations = 20;

int main(int argc, char** argv) {
  variables_and_factors_2d_registerTypes();
  solver_registerTypes();
  // linear_solver_registerTypes();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(DUMMY_DATA, SE2Plane2PlaneErrorFactor) {
  using VariableType    = VariableSE2Right;
  using VariablePtrType = std::shared_ptr<VariableType>;
  using FactorType      = SE2Plane2PlaneErrorFactorCorrespondenceDriven;
  using FactorPtrType   = std::shared_ptr<FactorType>;
  using SolverType      = Solver;

  const Vector3f minimal_T = 10 * Vector3f::Random();
  const Isometry2f T       = geometry2d::v2t(minimal_T);

  PointNormal2fVectorCloud fixed_cloud;
  PointNormal2fVectorCloud moving_cloud;
  CorrespondenceVector correspondences;
  fixed_cloud.reserve(n_meas);
  moving_cloud.reserve(n_meas);
  correspondences.reserve(n_meas);
  for (size_t m = 0; m < n_meas; ++m) {
    PointNormal2f fixed_point, moving_point;
    fixed_point.coordinates().setRandom();
    fixed_point.normal().setRandom();
    fixed_point.normal().normalize();
    moving_point = fixed_point.transform<TRANSFORM_CLASS::Isometry>(T);
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
  pose->setEstimate(Isometry2f::Identity());
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
  const Isometry2f diff_T    = estimated_T * T;
  const Vector3f diff_vector = geometry2d::t2v(diff_T);

  ASSERT_LT(diff_vector.x(), 1e-5);
  ASSERT_LT(diff_vector.y(), 1e-5);
  ASSERT_LT(diff_vector.z(), 1e-5);
  LOG << stats << std::endl;
}

TEST(DUMMY_DATA, SE2Plane2PlaneWithSensorErrorFactor) {
  using VariableType    = VariableSE2Right;
  using VariablePtrType = std::shared_ptr<VariableType>;
  using FactorType      = SE2Plane2PlaneWithSensorErrorFactorCorrespondenceDriven;
  using FactorPtrType   = std::shared_ptr<FactorType>;
  using SolverType      = Solver;

  const Vector3f t           = 10 * Vector3f::Random();
  Vector3f sensor_in_robot_v = Vector3f(0.1, 0.2, 0.5);
  Isometry2f sensor_in_robot = geometry2d::v2t(sensor_in_robot_v);
  Isometry2f ground_truth_T  = geometry2d::v2t(t);
  Isometry2f T               = ground_truth_T * sensor_in_robot;

  PointNormal2fVectorCloud fixed_cloud;
  PointNormal2fVectorCloud moving_cloud;
  CorrespondenceVector correspondences;
  fixed_cloud.reserve(n_meas);
  moving_cloud.reserve(n_meas);
  correspondences.reserve(n_meas);
  for (size_t m = 0; m < n_meas; ++m) {
    PointNormal2f fixed_point, moving_point;
    fixed_point.coordinates().setRandom();
    fixed_point.normal().setRandom();
    fixed_point.normal().normalize();
    moving_point = fixed_point.transform<TRANSFORM_CLASS::Isometry>(T);
    fixed_cloud.emplace_back(fixed_point);
    moving_cloud.emplace_back(moving_point);
    correspondences.emplace_back(m, m);
  }
  ASSERT_EQ(correspondences.size(), n_meas);

  SolverType solver;
  solver.param_max_iterations.pushBack(n_iterations);
  solver.param_termination_criteria.setValue(nullptr);
  FactorPtrType factor(new FactorType);
  factor->setVariableId(0, 0);
  factor->setFixed(fixed_cloud);
  factor->setMoving(moving_cloud);
  factor->setCorrespondences(correspondences);
  factor->setSensorInRobot(sensor_in_robot);

  FactorGraphPtr graph(new FactorGraph);
  graph->addFactor(factor);
  VariablePtrType pose = VariablePtrType(new VariableType);
  pose->setGraphId(0);
  pose->setEstimate(Isometry2f::Identity());
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
  const Isometry2f diff_T    = estimated_T * ground_truth_T;
  const Vector3f diff_vector = geometry2d::t2v(diff_T);

  LOG << stats << std::endl;

  LOG << "sensor T : " << geometry2d::t2v(T).transpose() << std::endl;
  LOG << "estim  T : " << geometry2d::t2v(estimated_T.inverse()).transpose() << std::endl;
  LOG << "GT       : " << geometry2d::t2v(ground_truth_T).transpose() << std::endl;

  ASSERT_LT(diff_vector.x(), 1e-5);
  ASSERT_LT(diff_vector.y(), 1e-5);
  ASSERT_LT(diff_vector.z(), 1e-5);
}
