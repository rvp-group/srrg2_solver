#include <gtest/gtest.h>

#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>

#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/internals/linear_solvers/instances.h"
#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/variables_and_factors/types_2d/instances.h"

const std::string exe_name = "test_se2_icp";
#define LOG std::cerr << exe_name + "|"

using namespace srrg2_core;
using namespace srrg2_solver;

const size_t n_meas       = 1000;
const size_t n_iterations = 10;

int main(int argc, char** argv) {
  registerTypes2D();
  solver_registerTypes();
  linear_solver_registerTypes();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(DUMMY_DATA, SE2Point2PointErrorFactor) {
  using VariableType     = VariableSE2Right;
  using VariablePtrType  = std::shared_ptr<VariableType>;
  using FactorType       = SE2Point2PointErrorFactorCorrespondenceDriven;
  using FactorPtrType    = std::shared_ptr<FactorType>;
  const Vector3f t       = 10 * Vector3f::Random();
  Isometry2f T           = geometry2d::v2t(t);
  const Isometry2f inv_T = T.inverse();

  // declare a multi_solver
  Solver solver;

  // declare a graph,
  FactorGraphPtr graph(new FactorGraph);

  // create a variable, set an id and add it to the graph
  VariablePtrType pose(new VariableType);
  pose->setGraphId(0);
  graph->addVariable(pose);

  graph->bindFactors();
  solver.setGraph(graph);

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
  // create an ICP factor, correspondence driven, set the var index, and add it to the graph
  FactorPtrType factor(new FactorType);
  factor->setVariableId(0, 0);
  graph->addFactor(factor);

  // setup the factor;
  factor->setFixed(fixed_cloud);
  factor->setMoving(moving_cloud);
  factor->setCorrespondences(correspondences);
  factor->setInformationMatrix(Matrix2f::Identity());

  solver.param_max_iterations.pushBack(n_iterations);
  solver.param_termination_criteria.setValue(nullptr);
  ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));

  pose->setEstimate(Isometry2f::Identity());
  solver.compute();
  const auto& stats      = solver.iterationStats();
  const auto& final_chi2 = stats.back().chi_inliers;

  // ia assert performed iterations are the effectively n_iterations
  ASSERT_EQ(stats.size(), n_iterations);
  // ia assert chi2 is good
  ASSERT_LT(final_chi2, 1e-6);
  // ia assert that relative error is good
  const auto& estimated_T    = pose->estimate();
  const Isometry2f diff_T    = estimated_T.inverse() * T;
  const Vector3f diff_vector = geometry2d::t2v(diff_T);
  LOG << stats << std::endl;

  ASSERT_LT(diff_vector.x(), 1e-5);
  ASSERT_LT(diff_vector.y(), 1e-5);
  ASSERT_LT(diff_vector.z(), 1e-5);
}

TEST(DUMMY_DATA, SE2Point2PointWithSensorErrorFactorAD) {
  using VariableType         = VariableSE2RightAD;
  using VariablePtrType      = std::shared_ptr<VariableType>;
  using FactorType           = SE2Point2PointWithSensorErrorFactorADCorrespondenceDriven;
  using FactorPtrType        = std::shared_ptr<FactorType>;
  const Vector3f t           = 10 * Vector3f::Random();
  Vector3f sensor_in_robot_v = Vector3f(0.1, 0.2, 0.5);
  Isometry2f sensor_in_robot = geometry2d::v2t(sensor_in_robot_v);
  Isometry2f ground_truth_T  = geometry2d::v2t(t);
  Isometry2f T               = ground_truth_T * sensor_in_robot;
  const Isometry2f inv_T     = T.inverse();

  // declare a multi_solver
  Solver solver;

  // declare a graph,
  FactorGraphPtr graph(new FactorGraph);

  // create a variable, set an id and add it to the graph
  VariablePtrType pose(new VariableType);
  pose->setGraphId(0);
  pose->setEstimate(Isometry2f::Identity());

  graph->addVariable(pose);

  graph->bindFactors();
  solver.setGraph(graph);

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
    moving_point.coordinates().setRandom();
    fixed_point.coordinates() = inv_T * moving_point.coordinates();
    fixed_cloud.emplace_back(fixed_point);
    moving_cloud.emplace_back(moving_point);
    correspondences.emplace_back(Correspondence(i, i));
  }
  // create an ICP factor, correspondence driven, set the var index, and add it to the graph
  FactorPtrType factor(new FactorType);
  factor->setVariableId(0, 0);
  graph->addFactor(factor);

  // setup the factor;
  factor->setFixed(fixed_cloud);
  factor->setMoving(moving_cloud);
  factor->setCorrespondences(correspondences);
  factor->setInformationMatrix(Matrix2f::Identity());
  factor->setSensorInRobot(sensor_in_robot);

  solver.param_max_iterations.pushBack(n_iterations);
  solver.param_termination_criteria.setValue(nullptr);
  ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));

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
