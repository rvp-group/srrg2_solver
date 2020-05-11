#include <gtest/gtest.h>

#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>

// ia include solver stuff (instances)
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/solver.h"
// ia include types stuff (instances)
#include "srrg_solver/variables_and_factors/types_3d/instances.h"

const std::string exe_name = "test_se3_icp";
#define LOG std::cerr << exe_name + "|"

using namespace srrg2_core;
using namespace srrg2_solver;

// ia global data
const size_t n_meas       = 1000;
const size_t n_iterations = 10;

int main(int argc, char** argv) {
  variables_and_factors_3d_registerTypes();
  solver_registerTypes();
  // linear_solver_registerTypes();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(DUMMY_DATA, SE3Point2PointErrorFactor) {
  using VariableType    = VariableSE3QuaternionRight;
  using VariablePtrType = std::shared_ptr<VariableType>;
  using FactorType      = SE3Point2PointErrorFactorCorrespondenceDriven;
  using FactorPtrType   = std::shared_ptr<FactorType>;

  const Vector3f robot_in_world_t = 10 * Vector3f::Random();
  Quaternionf robot_in_world_q(0.3, 0.3, 0.3, 0.3);
  robot_in_world_q.normalize();

  Isometry3f robot_in_world    = Isometry3f::Identity();
  robot_in_world.linear()      = robot_in_world_q.toRotationMatrix();
  robot_in_world.translation() = robot_in_world_t;

  // declare a multi_solver
  Solver solver;

  // declare a graph,
  FactorGraphPtr graph(new FactorGraph);

  // create a variable, set an id and add it to the graph
  VariablePtrType pose(new VariableType);
  pose->setGraphId(0);
  graph->addVariable(pose);

  // create an ICP factor, correspondence driven, set the var index, and add it to the graph
  FactorPtrType factor(new FactorType);
  factor->setVariableId(0, 0);
  graph->addFactor(factor);

  graph->bindFactors();
  solver.setGraph(graph);

  // ia create two cloud that we want to align, and a vector of correspondences
  Point3fVectorCloud fixed_cloud;
  Point3fVectorCloud moving_cloud;
  CorrespondenceVector correspondences;

  fixed_cloud.reserve(n_meas);
  moving_cloud.reserve(n_meas);
  correspondences.reserve(n_meas);
  for (size_t i = 0; i < n_meas; ++i) {
    // ia create dummy measurements
    Point3f fixed_point, moving_point;
    fixed_point.coordinates().setRandom();
    moving_point = fixed_point.transform<TRANSFORM_CLASS::Isometry>(robot_in_world);
    fixed_cloud.emplace_back(fixed_point);
    moving_cloud.emplace_back(moving_point);
    correspondences.emplace_back(Correspondence(i, i));
  }
  ASSERT_EQ(correspondences.size(), n_meas);

  // setup the factor;
  factor->setFixed(fixed_cloud);
  factor->setMoving(moving_cloud);
  factor->setCorrespondences(correspondences);
  factor->setInformationMatrix(Matrix3f::Identity());

  solver.param_max_iterations.pushBack(n_iterations);
  solver.param_termination_criteria.setValue(nullptr);
  ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));
  Isometry3f init_guess = Isometry3f::Identity();
  pose->setEstimate(init_guess);
  solver.compute();
  const auto& stats      = solver.iterationStats();
  const auto& final_chi2 = stats.back().chi_inliers;

  // ia assert performed iterations are the effectively n_iterations
  ASSERT_EQ(stats.size(), n_iterations);
  // ia assert chi2 is good
  ASSERT_LT(final_chi2, 1e-6);
  // ia assert that relative error is good
  const auto& estimated_T    = pose->estimate();
  const Isometry3f diff_T    = estimated_T * robot_in_world;
  const Vector6f diff_vector = geometry3d::t2v(diff_T);
  LOG << stats << std::endl;

  LOG << "init_guess         : " << geometry3d::t2v(init_guess.inverse()).transpose() << std::endl;
  LOG << "robot_in_world_est : " << geometry3d::t2v(estimated_T.inverse()).transpose() << std::endl;
  LOG << "robot_in_world     : " << geometry3d::t2v(robot_in_world).transpose() << std::endl;

  ASSERT_LT(diff_vector.x(), 1e-5);
  ASSERT_LT(diff_vector.y(), 1e-5);
  ASSERT_LT(diff_vector.z(), 1e-5);
}

TEST(DUMMY_DATA, SE3Point2PointWithSensorErrorFactor) {
  using VariableType    = VariableSE3QuaternionRight;
  using VariablePtrType = std::shared_ptr<VariableType>;
  using FactorType      = SE3Point2PointWithSensorErrorFactorCorrespondenceDriven;
  using FactorPtrType   = std::shared_ptr<FactorType>;

  const Vector3f robot_in_world_t = 10 * Vector3f::Random();
  Quaternionf robot_in_world_q(0.3, 0.3, 0.3, 0.3);
  robot_in_world_q.normalize();

  Isometry3f robot_in_world    = Isometry3f::Identity();
  robot_in_world.linear()      = robot_in_world_q.toRotationMatrix();
  robot_in_world.translation() = robot_in_world_t;

  const Vector3f sensor_in_robot_t = Vector3f::Random();
  Quaternionf sensor_in_robot_q(0.1, 0.1, 0.1, 0.1);
  sensor_in_robot_q.normalize();

  Isometry3f sensor_in_robot    = Isometry3f::Identity();
  sensor_in_robot.linear()      = sensor_in_robot_q.toRotationMatrix();
  sensor_in_robot.translation() = sensor_in_robot_t;

  Isometry3f sensor_in_world = robot_in_world * sensor_in_robot;

  // declare a multi_solver
  Solver solver;

  // declare a graph,
  FactorGraphPtr graph(new FactorGraph);

  // create a variable, set an id and add it to the graph
  VariablePtrType pose(new VariableType);
  pose->setGraphId(0);
  graph->addVariable(pose);

  // create an ICP factor, correspondence driven, set the var index, and add it to the graph
  FactorPtrType factor(new FactorType);
  factor->setVariableId(0, 0);
  graph->addFactor(factor);

  graph->bindFactors();
  solver.setGraph(graph);

  // ia create two cloud that we want to align, and a vector of correspondences
  Point3fVectorCloud fixed_cloud;
  Point3fVectorCloud moving_cloud;
  CorrespondenceVector correspondences;

  fixed_cloud.reserve(n_meas);
  moving_cloud.reserve(n_meas);
  correspondences.reserve(n_meas);
  for (size_t i = 0; i < n_meas; ++i) {
    // ia create dummy measurements
    Point3f fixed_point, moving_point;
    fixed_point.coordinates().setRandom();
    moving_point = fixed_point.transform<TRANSFORM_CLASS::Isometry>(sensor_in_world);
    fixed_cloud.emplace_back(fixed_point);
    moving_cloud.emplace_back(moving_point);
    correspondences.emplace_back(Correspondence(i, i));
  }
  ASSERT_EQ(correspondences.size(), n_meas);

  // setup the factor;
  factor->setFixed(fixed_cloud);
  factor->setMoving(moving_cloud);
  factor->setCorrespondences(correspondences);
  factor->setSensorInRobot(sensor_in_robot);
  factor->setInformationMatrix(Matrix3f::Identity());

  solver.param_max_iterations.pushBack(n_iterations);
  solver.param_termination_criteria.setValue(nullptr);
  ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));
  Isometry3f init_guess = Isometry3f::Identity();
  pose->setEstimate(init_guess);
  solver.compute();
  const auto& stats      = solver.iterationStats();
  const auto& final_chi2 = stats.back().chi_inliers;

  // ia assert performed iterations are the effectively n_iterations
  ASSERT_EQ(stats.size(), n_iterations);
  // ia assert chi2 is good
  ASSERT_LT(final_chi2, 1e-6);
  // ia assert that relative error is good
  const auto& estimated_T    = pose->estimate();
  const Isometry3f diff_T    = estimated_T * robot_in_world;
  const Vector6f diff_vector = geometry3d::t2v(diff_T);
  LOG << stats << std::endl;

  LOG << "init_guess         : " << geometry3d::t2v(init_guess.inverse()).transpose() << std::endl;
  LOG << "robot_in_world_est : " << geometry3d::t2v(estimated_T.inverse()).transpose() << std::endl;
  LOG << "robot_in_world     : " << geometry3d::t2v(robot_in_world).transpose() << std::endl;

  ASSERT_LT(diff_vector.x(), 1e-5);
  ASSERT_LT(diff_vector.y(), 1e-5);
  ASSERT_LT(diff_vector.z(), 1e-5);
}
