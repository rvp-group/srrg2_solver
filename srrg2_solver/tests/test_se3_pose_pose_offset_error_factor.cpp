#include <random>
#include <vector>

#include <gtest/gtest.h>

#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>

// ldg include solver stuff (instances)
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/solver.h"

// ldg include types stuff (instances)
#include "srrg_solver/variables_and_factors/types_3d/instances.h"

const std::string exe_name = "test_se3_pose_pose_offset_error_factor_ad";
#define LOG std::cerr << exe_name + "|"

using namespace srrg2_core;
using namespace srrg2_solver;

// ldg global data
const size_t n_poses      = 500;
const size_t n_iterations = 25;

using Isometry3fVector = std::vector<Isometry3f, Eigen::aligned_allocator<Isometry3f>>;

void createTrajectories(const size_t& n_poses_,
                        const Isometry3f& sensor_in_robot_,
                        Isometry3fVector& robot_trajectory_,
                        Isometry3fVector& sensor_trajectory_);

template <typename VariableType_, typename FactorType_>
void createFactorGraph(const Isometry3fVector& robot_trajectory_,
                       const Isometry3fVector& sensor_trajectory_,
                       const FactorGraphPtr& graph_);

int main(int argc, char** argv) {
  variables_and_factors_3d_registerTypes();
  solver_registerTypes();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(DUMMY_DATA, SE3PosePoseOffsetQuaternionErrorFactorAD) {
  // ldg types for this test
  using VType = VariableSE3QuaternionRightAD;
  using FType = SE3PosePoseOffsetErrorFactorAD;

  // ldg create a constant offset, gt pose
  Vector6f offset_vec;
  offset_vec << 0.1f, 0.2f, 0.f, 0.01f, -0.01f, 0.f;
  const Isometry3f sensor_in_robot = geometry3d::ta2t(offset_vec);

  // ldg create two trajectories with a constant offset
  Isometry3fVector robot_trajectory, sensor_trajectory;
  createTrajectories(n_poses, sensor_in_robot, robot_trajectory, sensor_trajectory);
  ASSERT_EQ(robot_trajectory.size(), sensor_trajectory.size());

  // ldg create a graph
  FactorGraphPtr graph(new FactorGraph);
  createFactorGraph<VType, FType>(robot_trajectory, sensor_trajectory, graph);

  // ldg instantiate solver, damping fundamental otherwise H not positive definite
  Solver solver;
  std::dynamic_pointer_cast<IterationAlgorithmGN>(solver.param_algorithm.value())
    ->param_damping.setValue(1.f);
  solver.param_termination_criteria.setValue(nullptr);
  solver.param_max_iterations.pushBack(n_iterations);
  solver.setGraph(graph);

  // ldg optimization step
  solver.compute();
  const auto& stats = solver.iterationStats();

  const auto& final_chi2 = stats.back().chi_inliers;
  ASSERT_EQ(stats.size(), n_iterations);
  LOG << "stats:\n\n" << stats << std::endl;

  // ldg assert that chi2 is good
  ASSERT_LT(final_chi2, 1e-6);

  // ldg validate transforms and boxminus
  const auto& final_estimated_pose = static_cast<VType*>(graph->variable(0))->estimate();
  const auto& vec_diff = geometry3d::t2v(final_estimated_pose.inverse() * sensor_in_robot);
  LOG << "GT: " << geometry3d::t2v(sensor_in_robot).transpose() << std::endl;
  LOG << "Estimate: " << geometry3d::t2v(final_estimated_pose).transpose() << std::endl;

  ASSERT_LT(vec_diff[0], 1e-6);
  ASSERT_LT(vec_diff[1], 1e-6);
  ASSERT_LT(vec_diff[2], 1e-6);
  ASSERT_LT(vec_diff[3], 1e-6);
  ASSERT_LT(vec_diff[4], 1e-6);
  ASSERT_LT(vec_diff[5], 1e-6);
}

void createTrajectories(const size_t& n_poses_,
                        const Isometry3f& sensor_in_robot_,
                        Isometry3fVector& robot_trajectory_,
                        Isometry3fVector& sensor_trajectory_) {
  robot_trajectory_.clear();
  sensor_trajectory_.clear();
  robot_trajectory_.reserve(n_poses_);
  sensor_trajectory_.reserve(n_poses_);

  std::mt19937 rnd_generator;
  std::uniform_real_distribution<float> uniform_distribution(0.0, 1.0);

  Isometry3f previous_pose              = Isometry3f::Identity();
  const Isometry3f previous_sensor_pose = previous_pose * sensor_in_robot_;
  robot_trajectory_.emplace_back(previous_pose);
  sensor_trajectory_.emplace_back(previous_sensor_pose);

  for (size_t i = 1; i < n_poses_; ++i) {
    const Vector6f current_pose = geometry3d::t2ta(previous_pose);
    Vector6f next_pose          = Vector6f::Zero();

    const float dir_selector  = uniform_distribution(rnd_generator);
    const float current_theta = current_pose[5];
    float next_theta = 0, next_x = 0, next_y = 0;

    if (dir_selector < 0.6) {
      next_x = std::round(std::cos(current_theta));
      next_y = std::round(std::sin(current_theta));
    } else if (dir_selector < 0.75 && 0.6 < dir_selector) {
      next_x     = std::round(-std::sin(current_theta));
      next_y     = std::round(std::cos(current_theta));
      next_theta = M_PI / 2.0f;
    } else {
      next_x     = std::round(std::sin(current_theta));
      next_y     = std::round(-std::cos(current_theta));
      next_theta = -M_PI / 2.0f;
    }

    next_pose.head(2) = current_pose.head(2) + Vector2f(next_x, next_y);
    next_pose[5]      = current_pose[5] + next_theta;

    // ldg next_T is robot in world transform
    const Isometry3f next_T = geometry3d::ta2t(next_pose);
    robot_trajectory_.emplace_back(next_T);

    // ldg we multiply this by a constant offset, this will give
    // ldg constant offset trajectory, which is the one made by the sensor
    const Isometry3f sensor_in_world = next_T * sensor_in_robot_;
    sensor_trajectory_.emplace_back(sensor_in_world);

    previous_pose = next_T;
  }
}

template <typename VariableType_, typename FactorType_>
void createFactorGraph(const Isometry3fVector& robot_trajectory_,
                       const Isometry3fVector& sensor_trajectory_,
                       const FactorGraphPtr& graph_) {
  const int OFFSET_ID = 0;
  std::shared_ptr<VariableType_> offset(new VariableType_);
  offset->setEstimate(Isometry3f::Identity()); // ldg initial guess
  offset->setGraphId(OFFSET_ID);
  graph_->addVariable(offset);

  // ldg create factor graph
  for (size_t i = 1; i < robot_trajectory_.size(); ++i) {
    const auto& curr_pose        = robot_trajectory_[i];
    const auto& prev_pose        = robot_trajectory_[i - 1];
    const auto& curr_sensor_pose = sensor_trajectory_[i];
    const auto& prev_sensor_pose = sensor_trajectory_[i - 1];

    // ldg create factor
    std::shared_ptr<FactorType_> factor(new FactorType_);
    factor->setVariableId(0, offset->graphId());
    factor->setFrom(prev_pose);
    factor->setTo(curr_pose);
    // ldg add measurament
    Isometry3f Z_sensor = prev_sensor_pose.inverse() * curr_sensor_pose;
    factor->setMeasurement(Z_sensor);
    graph_->addFactor(factor);
  }

  graph_->bindFactors();
}
