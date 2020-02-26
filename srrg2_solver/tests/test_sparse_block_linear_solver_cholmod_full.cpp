#include <random>
#include <vector>

#include <gtest/gtest.h>

#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>

#include "srrg_solver/solver_core/internals/linear_solvers/instances.h"
#include "srrg_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholmod_full.h"
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/variables_and_factors/types_3d/instances.h"

const std::string exe_name = "test_sparse_block_linear_solver_cholmod_full";
#define LOG std::cerr << exe_name + "|"

using namespace srrg2_core;
using namespace srrg2_solver;

// ia global data
const float sensing_radius = 2.5f;
const size_t n_poses       = 200;
const size_t n_iterations  = 25;

using FactorType       = SE3PosePoseGeodesicErrorFactor;
using Isometry3fVector = std::vector<Isometry3f, Eigen::aligned_allocator<Isometry3f>>;

void createGTTrajectory(const size_t& n_poses_, Isometry3fVector& gt_trajectory_);
void createFactorGraph(const Isometry3fVector& gt_trajectory_,
                         const FactorGraphPtr& graph_,
                         const bool enable_closure_);

int main(int argc, char** argv) {
  registerTypes3D();
  solver_registerTypes();
  linear_solver_registerTypes();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(SYNTHETIC_DATA, SparseBlockLinearSolverCholmodFull) {
  Isometry3fVector gt_trajectory;
  createGTTrajectory(n_poses, gt_trajectory);

  // ia create a graph
  FactorGraphPtr graph(new FactorGraph);
  createFactorGraph(gt_trajectory, graph, true);

  // ia bind factors and variables
  graph->bindFactors();

  const size_t n_variables = graph->variables().size();
  ASSERT_EQ(n_variables, n_poses);

  for (size_t i = 1; i < n_variables; ++i) {
    static_cast<VariableSE3QuaternionRight*>(graph->variable(i))->setEstimate(Isometry3f::Identity());
  }

  Solver solver;
  solver.param_termination_criteria.setValue(nullptr);
  solver.param_linear_solver.setValue(
    std::shared_ptr<SparseBlockLinearSolverCholmodFull>(new SparseBlockLinearSolverCholmodFull));
  solver.param_max_iterations.pushBack(n_iterations);
  solver.setGraph(graph);

  // ia do the optimization
  solver.compute();
  const auto& stats      = solver.iterationStats();
  const auto& final_chi2 = stats.back().chi_inliers;
  ASSERT_EQ(stats.size(), n_iterations);
  LOG << "stats:\n\n" << stats << std::endl;

  // ia assert that chi2 is ok
  ASSERT_LT(final_chi2, 1e-6);

  // ia assert final pose is the same as the original one
  const auto& final_estimated_pose =
    static_cast<VariableSE3QuaternionRight*>(graph->variable(n_poses - 1))->estimate();
  const auto& final_gt_pose = gt_trajectory[n_poses - 1];
  const auto diff           = geometry3d::t2tnq(final_estimated_pose.inverse() * final_gt_pose);
  ASSERT_LT(diff[0], 1e-5); // ia X
  ASSERT_LT(diff[1], 1e-5); // ia Y
  ASSERT_LT(diff[2], 1e-5); // ia Z
  ASSERT_LT(diff[3], 1e-5); // ia qx
  ASSERT_LT(diff[4], 1e-5); // ia qy
  ASSERT_LT(diff[5], 1e-5); // ia qz
}

void createGTTrajectory(const size_t& n_poses_, Isometry3fVector& gt_trajectory_) {
  gt_trajectory_.clear();
  gt_trajectory_.reserve(n_poses_);

  std::mt19937 rnd_generator;
  std::uniform_real_distribution<float> uniform_distribution(0.0, 1.0);

  Isometry3f previous_pose = Isometry3f::Identity();
  gt_trajectory_.emplace_back(previous_pose);

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

    const Isometry3f next_T = geometry3d::ta2t(next_pose);
    gt_trajectory_.emplace_back(next_T);

    previous_pose = next_T;
  }
}

void createFactorGraph(const Isometry3fVector& gt_trajectory_,
                         const FactorGraphPtr& graph_,
                         const bool enable_closure_) {
  VariableSE3QuaternionRight* current_variable = new VariableSE3QuaternionRight();
  current_variable->setGraphId(0);
  current_variable->setEstimate(gt_trajectory_[0]);
  current_variable->setStatus(VariableBase::Status::Fixed);
  graph_->addVariable(VariableBasePtr(current_variable));

  for (size_t i = 1; i < gt_trajectory_.size(); ++i) {
    const size_t curr_id     = i;
    const auto& current_pose = gt_trajectory_[curr_id];

    current_variable = new VariableSE3QuaternionRight();
    current_variable->setGraphId(i);
    current_variable->setEstimate(current_pose);
    graph_->addVariable(VariableBasePtr(current_variable));

    const size_t prev_id = i - 1;
    VariableSE3QuaternionRight* prev_variable =
      static_cast<VariableSE3QuaternionRight*>(graph_->variable(prev_id));
    const Isometry3f Z_odom_gt = prev_variable->estimate().inverse() * current_variable->estimate();
    const Matrix6f odom_information = Matrix6f::Identity();

    FactorType* odom_factor = new FactorType();
    odom_factor->setVariableId(0, prev_variable->graphId());
    odom_factor->setVariableId(1, current_variable->graphId());
    odom_factor->setMeasurement(Z_odom_gt);
    odom_factor->setInformationMatrix(odom_information);
    graph_->addFactor(FactorBasePtr(odom_factor));

    if (!enable_closure_) {
      continue;
    }

    for (const auto& id_var_pair : graph_->variables()) {
      if ((size_t)(id_var_pair.first) == curr_id) {
        continue;
      }

      const VariableSE3QuaternionRight* var =
        static_cast<const VariableSE3QuaternionRight*>(id_var_pair.second);
      const Isometry3f delta      = var->estimate().inverse() * current_variable->estimate();
      const Vector6f delta_vector = geometry3d::t2tnq(delta);
      const Vector3f delta_trans  = delta_vector.head(3);
      if (delta_trans.norm() < sensing_radius) {
        const Matrix6f closure_information = Matrix6f::Identity();
        FactorType* closure_factor         = new FactorType();
        closure_factor->setVariableId(0, var->graphId());
        closure_factor->setVariableId(1, current_variable->graphId());
        closure_factor->setMeasurement(delta);
        closure_factor->setInformationMatrix(closure_information);
        graph_->addFactor(FactorBasePtr(closure_factor));
      }
    }
  }
}