#include <gtest/gtest.h>
#include <random>
#include <vector>

#include <srrg_geometry/similiarity.hpp>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>

// ldg include solver stuff (instances)
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/solver.h"

// ldg include variable stuff
#include "srrg_solver/variables_and_factors/types_projective/instances.h"
#include "srrg_solver/variables_and_factors/types_projective/sim3_pose_pose_error_factor_ad.h"
#include "srrg_solver/variables_and_factors/types_projective/variable_sim3.hpp"

const std::string exe_name = "test_sim3_pgo";
#define LOG std::cerr << exe_name + "|"

using namespace srrg2_core;
using namespace srrg2_solver;

// ldg global data
const float sensing_radius = 2.5f;
const size_t n_poses       = 100;
const size_t n_iterations  = 30;

using Similiarity3fVector = std::vector<Similiarity3f, Eigen::aligned_allocator<Similiarity3f>>;

void createGTTrajectory(const size_t& n_poses_, Similiarity3fVector& gt_trajectory_);
template <typename PGOVariableType, typename PGOFactorType_>
void createFactorGraph(const Similiarity3fVector& gt_trajectory_, const FactorGraphPtr& graph_);

int main(int argc, char** argv) {
  solver_registerTypes();
  variables_and_factors_projective_registerTypes();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(DUMMY_DATA, Sim3PosePoseErrorFactorAD) {
  // ldg types for similiarity test
  using VType = VariableSim3QuaternionRightAD;
  using FType = Sim3PosePoseErrorFactorAD;

  Similiarity3fVector gt_trajectory;
  createGTTrajectory(n_poses, gt_trajectory);

  // ldg create a graph
  FactorGraphPtr graph(new FactorGraph);
  createFactorGraph<VType, FType>(gt_trajectory, graph);

  // ldg bind factors and variables
  graph->bindFactors();

  const size_t n_variables = graph->variables().size();
  ASSERT_EQ(n_variables, n_poses);

  std::mt19937 rnd_generator;
  const float stddv = 0.1f;
  std::uniform_real_distribution<float> uniform_distribution(0.0, stddv);

  for (size_t i = 1; i < n_variables; ++i) {
    // ldg bad initial guess
    static_cast<VType*>(graph->variable(i))->setEstimate(Similiarity3f::Identity());
  }

  Solver solver;
  solver.param_termination_criteria.setValue(nullptr);
  solver.param_max_iterations.pushBack(n_iterations);
  solver.setGraph(graph);

  // ldg optimization
  solver.compute();
  const auto& stats      = solver.iterationStats();
  const auto& final_chi2 = stats.back().chi_inliers;
  ASSERT_EQ(stats.size(), n_iterations);
  LOG << "stats:\n\n" << stats << std::endl;

  // ldg assert that chi2 is ok
  ASSERT_LT(final_chi2, 1e-6);

  // ldg assert final pose is the same as the original one
  const auto& final_estimated_pose = static_cast<VType*>(graph->variable(n_poses - 1))->estimate();
  const auto& final_gt_pose        = gt_trajectory[n_poses - 1];

  // ldg output last poses, gt and estimate to check
  std::cerr << "Final Estimated Pose: " << geometry3d::s2v(final_estimated_pose).transpose()
            << std::endl;
  std::cerr << "Final GT pose: " << geometry3d::s2v(final_gt_pose).transpose() << std::endl;

  // ldg boxminus to get difference between poses
  const auto diff = geometry3d::s2v(final_estimated_pose.inverse() * final_gt_pose);
  ASSERT_LT(diff[0], 1e-5); //  tx
  ASSERT_LT(diff[1], 1e-5); //  ty
  ASSERT_LT(diff[2], 1e-5); //  tz
  ASSERT_LT(diff[3], 1e-5); //  rx
  ASSERT_LT(diff[4], 1e-5); //  ry
  ASSERT_LT(diff[5], 1e-5); //  rz
  ASSERT_LT(diff[6], 1e-5); //  s
}

void createGTTrajectory(const size_t& n_poses_, Similiarity3fVector& gt_trajectory_) {
  gt_trajectory_.clear();
  gt_trajectory_.reserve(n_poses_);

  std::mt19937 rnd_generator;
  std::uniform_real_distribution<float> uniform_distribution(0.0, 1.0);

  Similiarity3f previous_pose = Similiarity3f::Identity();
  gt_trajectory_.emplace_back(previous_pose);

  for (size_t i = 1; i < n_poses_; ++i) {
    const Vector7f current_pose = geometry3d::s2v(previous_pose);
    Vector7f next_pose          = Vector7f::Zero();

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

    // ldg scaling factor incremental for each pose
    next_pose[6] = current_pose[6] + 0.01f;

    const Similiarity3f next_T = geometry3d::v2s(next_pose);
    gt_trajectory_.emplace_back(next_T);

    previous_pose = next_T;
  }
}

template <typename PGOVariableType_, typename PGOFactorType_>
void createFactorGraph(const Similiarity3fVector& gt_trajectory_, const FactorGraphPtr& graph_) {
  using OmegaType = typename PGOFactorType_::InformationMatrixType;

  // ldg add a prior fix the first variable
  PGOVariableType_* current_variable = new PGOVariableType_();
  current_variable->setGraphId(0);
  current_variable->setEstimate(gt_trajectory_[0]);
  current_variable->setStatus(VariableBase::Status::Fixed);
  graph_->addVariable(VariableBasePtr(current_variable));

  for (size_t i = 1; i < gt_trajectory_.size(); ++i) {
    const size_t curr_id     = i;
    const auto& current_pose = gt_trajectory_[curr_id];

    current_variable = new PGOVariableType_();
    current_variable->setGraphId(i);
    current_variable->setEstimate(current_pose);
    graph_->addVariable(VariableBasePtr(current_variable));

    const size_t prev_id            = i - 1;
    PGOVariableType_* prev_variable = static_cast<PGOVariableType_*>(graph_->variable(prev_id));
    const Similiarity3f Z_odom_gt =
      prev_variable->estimate().inverse() * current_variable->estimate();
    const OmegaType odom_information = OmegaType::Identity();
    // ldg create factor graph
    PGOFactorType_* odom_factor = new PGOFactorType_();
    odom_factor->setVariableId(0, prev_variable->graphId());
    odom_factor->setVariableId(1, current_variable->graphId());
    odom_factor->setMeasurement(Z_odom_gt);
    odom_factor->setInformationMatrix(odom_information);
    graph_->addFactor(FactorBasePtr(odom_factor));

    for (const auto& id_var_pair : graph_->variables()) {
      if ((size_t)(id_var_pair.first) == curr_id) {
        continue;
      }
      // ldg creating factor for closure
      const PGOVariableType_* var = static_cast<const PGOVariableType_*>(id_var_pair.second);
      const Similiarity3f delta   = var->estimate().inverse() * current_variable->estimate();
      const Vector7f delta_vector = geometry3d::s2v(delta);
      const Vector3f delta_trans  = delta_vector.head(3);
      if (delta_trans.norm() < sensing_radius) {
        const OmegaType closure_information = OmegaType::Identity();
        PGOFactorType_* closure_factor      = new PGOFactorType_();
        closure_factor->setVariableId(0, var->graphId());
        closure_factor->setVariableId(1, current_variable->graphId());
        closure_factor->setMeasurement(delta);
        closure_factor->setInformationMatrix(closure_information);
        graph_->addFactor(FactorBasePtr(closure_factor));
      }
    }
  }
}
