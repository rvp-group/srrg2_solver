#include <random>
#include <vector>

#include <gtest/gtest.h>

#include <srrg_solver/variables_and_factors/types_2d/instances.h>
#include <srrg_solver/solver_core/factor_graph.h>
#include <srrg_solver/solver_core/instances.h>
#include <srrg_solver/solver_core/solver.h>
#include "srrg_solver_extras/types_2d_ad/se2_pose_pose_left_error_factor_ad.h"
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>
using namespace srrg2_core;
using namespace srrg2_solver;
using namespace srrg2_solver_extras;

const std::string exe_name = "test_se2_pgo_left_ad";
#define LOG std::cerr << exe_name + "|"

size_t num_poses           = 200;
size_t num_iterations      = 20;
const float sensing_radius = 2.5f;

using Isometry2fVector = std::vector<Isometry2f, Eigen::aligned_allocator<Isometry2f>>;

// ia creates a perfect trajectory without noise
void createGTTrajectory(const size_t& n_poses_, Isometry2fVector& gt_trajectory_);

// ia creates a factor graph from the trajectory, including also lc
void createFactorGraph(const Isometry2fVector& gt_trajectory_,
                       const FactorGraphPtr& graph_,
                       const bool enable_closures_);

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(SYNTHETIC_DATA, SE2PosePoseLeftErrorFactorAD) {
  FactorGraphPtr graph(new FactorGraph);
  Isometry2fVector gt_trajectory;
  createGTTrajectory(num_poses, gt_trajectory);
  createFactorGraph(gt_trajectory, graph, true);

  graph->bindFactors();
  const size_t num_variables = graph->variables().size();
  ASSERT_EQ(num_variables, num_poses);

  for (size_t i = 1; i < num_variables; ++i) {
    static_cast<VariableSE2Left*>(graph->variable(i))->setEstimate(Isometry2f::Identity());
  }

  Solver solver;
  solver.param_termination_criteria.setValue(nullptr);
  solver.param_max_iterations.pushBack(num_iterations);
  solver.setGraph(graph);

  solver.compute();
  const auto& stats      = solver.iterationStats();
  const auto& final_chi2 = stats.back().chi_inliers;
  LOG << stats << std::endl;

  ASSERT_LT(final_chi2, 1e-6);

  const auto& final_estimated_pose =
    static_cast<VariableSE2Left*>(graph->variable(num_poses - 1))->estimate();
  const auto& final_gt_pose = gt_trajectory[num_poses - 1];
  const auto diff           = geometry2d::t2v(final_estimated_pose.inverse() * final_gt_pose);
  ASSERT_LT(diff[0], 1e-5);
  ASSERT_LT(diff[1], 1e-5);
  ASSERT_LT(diff[2], 1e-5);
}

void createGTTrajectory(const size_t& n_poses_, Isometry2fVector& gt_trajectory_) {
  gt_trajectory_.clear();
  gt_trajectory_.reserve(n_poses_);

  std::mt19937 rnd_generator;
  std::uniform_real_distribution<float> uniform_distribution(0.0, 1.0);

  Isometry2f previous_pose = Isometry2f::Identity();
  gt_trajectory_.emplace_back(previous_pose);

  for (size_t i = 1; i < n_poses_; ++i) {
    const Vector3f current_pose = geometry2d::t2v(previous_pose);
    Vector3f next_pose          = Vector3f::Zero();

    const float dir_selector  = uniform_distribution(rnd_generator);
    const float current_theta = current_pose[2];
    float next_theta = 0, next_x = 0, next_y = 0;

    if (dir_selector < 0.5) {
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
    next_pose[2]      = current_pose[2] + next_theta;

    const Isometry2f next_T = geometry2d::v2t(next_pose);
    gt_trajectory_.emplace_back(next_T);

    previous_pose = next_T;
  }
}

void createFactorGraph(const Isometry2fVector& gt_trajectory_,
                       const FactorGraphPtr& graph_,
                       const bool enable_closures_) {

  using FactorType = SE2PosePoseLeftErrorFactorAD;
  using VariableType = VariableSE2LeftAD;

  VariableType* current_variable = new VariableType;
  current_variable->setGraphId(0);
  current_variable->setEstimate(gt_trajectory_[0]);
  current_variable->setStatus(VariableBase::Status::Fixed);
  graph_->addVariable(VariableBasePtr(current_variable));

  for (size_t i = 1; i < gt_trajectory_.size(); ++i) {
    const size_t curr_id     = i;
    const auto& current_pose = gt_trajectory_[curr_id];

    current_variable = new VariableType;
    current_variable->setGraphId(i);
    current_variable->setEstimate(current_pose);
    graph_->addVariable(VariableBasePtr(current_variable));

    const size_t prev_id              = i - 1;
    VariableType* prev_variable = static_cast<VariableType*>(graph_->variable(prev_id));
    const Isometry2f Z_odom_gt = prev_variable->estimate().inverse() * current_variable->estimate();
    FactorType* odom_factor    = new FactorType();
    odom_factor->setVariableId(0, prev_variable->graphId());
    odom_factor->setVariableId(1, current_variable->graphId());
    odom_factor->setMeasurement(Z_odom_gt);
    graph_->addFactor(FactorBasePtr(odom_factor));

    if (!enable_closures_) {
      continue;
    }

    for (const auto& id_var_pair : graph_->variables()) {
      const VariableType* var = static_cast<const VariableType*>(id_var_pair.second);
      const Isometry2f delta        = var->estimate().inverse() * current_variable->estimate();
      const Vector3f delta_vector   = geometry2d::t2v(delta);
      const Vector2f delta_trans    = delta_vector.head(2);
      if (delta_trans.norm() < sensing_radius && var->graphId() != current_variable->graphId()) {
        FactorType* closure_factor = new FactorType();
        closure_factor->setVariableId(0, var->graphId());
        closure_factor->setVariableId(1, current_variable->graphId());
        closure_factor->setMeasurement(delta);
        graph_->addFactor(FactorBasePtr(closure_factor));
      }
    }
  }
}
