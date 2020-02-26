#include <random>
#include <vector>

#include <gtest/gtest.h>

#include <srrg_geometry/geometry3d.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>

#include <srrg_solver/solver_core/factor_graph.h>
#include <srrg_solver/solver_core/instances.h>
#include <srrg_solver/solver_core/solver.h>
#include <srrg_solver/variables_and_factors/types_3d/instances.h>
#include "srrg_solver_extras/types_3d_ad/se3_pose_point_euler_error_factor_ad.h"
#include "srrg_solver_extras/types_3d_ad/se3_pose_point_offset_euler_error_factor_ad.h"

const std::string exe_name = "test_se3_multi_point_registration";
#define LOG std::cerr << exe_name + "|"

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace srrg2_solver_extras;

size_t num_visible_landmarks_per_pose = 5;
size_t num_poses                      = 100;
size_t num_iterations                 = 20;
float perception_range                = 2.5f;
using Vector3fVector                  = std::vector<Vector3f, Eigen::aligned_allocator<Vector3f>>;
using Isometry3fVector = std::vector<Isometry3f, Eigen::aligned_allocator<Isometry3f>>;

void createGTTrajectory(const size_t& n_poses_,
                        const Isometry3f& offset_,
                        Isometry3fVector& gt_trajectory_,
                        Vector3fVector& landmarks);
int main(int argc, char** argv) {

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(SYNTHETIC_DATA, SE3PosePointOffsetEulerErrorFactorAD) {
  using PoseVariableType     = VariableSE3EulerRightAD;
  using PointVariableType    = VariablePoint3AD;
  using FactorType           = SE3PosePointOffsetEulerErrorFactorAD;
  using VariableSE3Vector    = std::vector<PoseVariableType*>;
  using VariablePoint3Vector = std::vector<PointVariableType*>;
  using FactorVector         = std::vector<FactorType*>;

  // tg containers
  VariableSE3Vector poses;
  poses.reserve(num_poses);
  VariablePoint3Vector landmarks;
  landmarks.reserve(num_poses * num_visible_landmarks_per_pose);

  Vector3f landmark_perturbation = Vector3f::Zero();

  FactorGraphPtr graph(new FactorGraph);
  PoseVariableType* v_offset = new PoseVariableType;
  Isometry3f offset          = Isometry3f::Identity();
  Vector4f q_norm            = Vector4f::Random();
  q_norm.normalize();
  Quaternionf q(q_norm(0), q_norm(1), q_norm(2), q_norm(3));
  offset.translation() = Vector3f::Random();
  offset.linear()      = q.toRotationMatrix();
  v_offset->setEstimate(offset);
  v_offset->setStatus(VariableBase::Status::Fixed);
  graph->addVariable(VariableBasePtr(v_offset));

  Isometry3fVector ground_truth_poses;
  Vector3fVector ground_truth_landmarks;

  createGTTrajectory(num_poses, offset, ground_truth_poses, ground_truth_landmarks);

  // tg create an anchor for the graph
  PoseVariableType* v = new PoseVariableType;
  const Isometry3f& T = ground_truth_poses.front();
  v->setEstimate(T);
  v->setStatus(VariableBase::Status::Fixed);
  poses.emplace_back(v);
  graph->addVariable(VariableBasePtr(v));

  for (size_t pose_idx = 1; pose_idx < num_poses; ++pose_idx) {
    v = new PoseVariableType;
    v->setEstimate(Isometry3f::Identity());
    poses.emplace_back(v);
    graph->addVariable(VariableBasePtr(v));
  }

  ASSERT_EQ(poses.size(), ground_truth_poses.size());
  ASSERT_EQ(poses.size(), num_poses);

  // tg generate random landmarks
  PointVariableType* point = nullptr;
  size_t num_landmarks     = ground_truth_landmarks.size();
  for (size_t landmark_idx = 0; landmark_idx < num_landmarks; ++landmark_idx) {
    landmark_perturbation.setRandom();
    point = new PointVariableType;
    landmarks.emplace_back(point);
    point->setEstimate(ground_truth_landmarks[landmark_idx] + landmark_perturbation);
    graph->addVariable(VariableBasePtr(point));
  }

  // tg all landmark are visible from every pose
  FactorType* f = nullptr;
  FactorVector factors;
  factors.reserve(num_poses * num_landmarks);
  for (size_t p = 0; p < num_poses; ++p) {
    const Isometry3f& pose = ground_truth_poses.at(p);
    for (size_t l = 0; l < num_landmarks; ++l) {
      const Vector3f& point = ground_truth_landmarks.at(l);
      if ((point - pose.translation()).norm() < perception_range) {
        f = new FactorType;
        f->setVariableId(0, poses[p]->graphId());
        f->setVariableId(1, landmarks[l]->graphId());
        f->setVariableId(2, v_offset->graphId());
        const Isometry3f total_transform = pose * offset;
        f->setMeasurement(total_transform.inverse() * point);
        graph->addFactor(FactorBasePtr(f));
        factors.emplace_back(f);
      }
    }
  }

  // tg optimize graph
  Solver solver;
  solver.param_max_iterations.pushBack(num_iterations);
  solver.param_termination_criteria.setValue(nullptr);
  solver.setGraph(graph);
  solver.compute();

  const auto& stats      = solver.iterationStats();
  const float chi_square = stats.back().chi_inliers;

  LOG << stats << std::endl;
  // tg assert chi square is good
  ASSERT_LT(chi_square, 1e-6);

  poses.clear();
  landmarks.clear();
}

TEST(SYNTHETIC_DATA, SE3PosePointEulerErrorFactorAD) {
  using PoseVariableType     = VariableSE3EulerRightAD;
  using PointVariableType    = VariablePoint3AD;
  using FactorType           = SE3PosePointEulerErrorFactorAD;
  using VariableSE3Vector    = std::vector<PoseVariableType*>;
  using VariablePoint3Vector = std::vector<PointVariableType*>;
  using FactorVector         = std::vector<FactorType*>;

  // tg containers
  VariableSE3Vector poses;
  poses.reserve(num_poses);
  VariablePoint3Vector landmarks;
  landmarks.reserve(num_poses * num_visible_landmarks_per_pose);

  Vector3f landmark_perturbation = Vector3f::Zero();

  FactorGraphPtr graph(new FactorGraph);
  PoseVariableType* v_offset = new PoseVariableType;
  Isometry3f offset          = Isometry3f::Identity();
  Vector4f q_norm            = Vector4f::Random();
  q_norm.normalize();
  Quaternionf q(q_norm(0), q_norm(1), q_norm(2), q_norm(3));
  offset.translation() = Vector3f::Random();
  offset.linear()      = q.toRotationMatrix();
  v_offset->setEstimate(offset);
  v_offset->setStatus(VariableBase::Status::Fixed);
  graph->addVariable(VariableBasePtr(v_offset));

  Isometry3fVector ground_truth_poses;
  Vector3fVector ground_truth_landmarks;

  createGTTrajectory(num_poses, offset, ground_truth_poses, ground_truth_landmarks);

  // tg create an anchor for the graph
  PoseVariableType* v = new PoseVariableType;
  const Isometry3f& T = ground_truth_poses.front();
  v->setEstimate(T);
  v->setStatus(VariableBase::Status::Fixed);
  poses.emplace_back(v);
  graph->addVariable(VariableBasePtr(v));

  for (size_t pose_idx = 1; pose_idx < num_poses; ++pose_idx) {
    v = new PoseVariableType;
    v->setEstimate(Isometry3f::Identity());
    poses.emplace_back(v);
    graph->addVariable(VariableBasePtr(v));
  }

  ASSERT_EQ(poses.size(), ground_truth_poses.size());
  ASSERT_EQ(poses.size(), num_poses);

  // tg generate random landmarks
  PointVariableType* point = nullptr;
  size_t num_landmarks     = ground_truth_landmarks.size();
  for (size_t landmark_idx = 0; landmark_idx < num_landmarks; ++landmark_idx) {
    landmark_perturbation.setRandom();
    point = new PointVariableType;
    landmarks.emplace_back(point);
    point->setEstimate(ground_truth_landmarks[landmark_idx] + landmark_perturbation);
    graph->addVariable(VariableBasePtr(point));
  }

  // tg all landmark are visible from every pose
  FactorType* f = nullptr;
  FactorVector factors;
  factors.reserve(num_poses * num_landmarks);
  for (size_t p = 0; p < num_poses; ++p) {
    const Isometry3f& pose = ground_truth_poses.at(p);
    for (size_t l = 0; l < num_landmarks; ++l) {
      const Vector3f& point = ground_truth_landmarks.at(l);
      if ((point - pose.translation()).norm() < perception_range) {
        f = new FactorType;
        f->setVariableId(0, poses[p]->graphId());
        f->setVariableId(1, landmarks[l]->graphId());
        const Isometry3f total_transform = pose * offset;
        f->setMeasurement(total_transform.inverse() * point);
        graph->addFactor(FactorBasePtr(f));
        factors.emplace_back(f);
      }
    }
  }

  // tg optimize graph
  Solver solver;
  solver.param_max_iterations.pushBack(num_iterations);
  solver.param_termination_criteria.setValue(nullptr);
  solver.setGraph(graph);
  solver.compute();

  const auto& stats      = solver.iterationStats();
  const float chi_square = stats.back().chi_inliers;

  LOG << stats << std::endl;
  // tg assert chi square is good
  ASSERT_LT(chi_square, 1e-6);

  poses.clear();
  landmarks.clear();
}

void createGTTrajectory(const size_t& n_poses_,
                        const Isometry3f& offset,
                        Isometry3fVector& gt_trajectory_,
                        Vector3fVector& landmarks_) {
  gt_trajectory_.clear();
  gt_trajectory_.reserve(n_poses_);
  landmarks_.clear();
  landmarks_.reserve(num_visible_landmarks_per_pose * n_poses_);
  std::mt19937 rnd_generator;
  std::uniform_real_distribution<float> uniform_distribution(0.0, 1.0);

  Isometry3f previous_pose = Isometry3f::Identity();
  gt_trajectory_.emplace_back(previous_pose);
  Vector3f point;
  Vector3f transformed_point;
  Isometry3f global_transform = previous_pose * offset;
  for (size_t land_idx = 0; land_idx < num_visible_landmarks_per_pose; ++land_idx) {
    point.setRandom();
    transformed_point = global_transform * point;
    landmarks_.emplace_back(transformed_point);
  }

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
    global_transform = next_T * offset;
    for (size_t land_idx = 0; land_idx < num_visible_landmarks_per_pose; ++land_idx) {
      point.setRandom();
      transformed_point = global_transform * point;
      landmarks_.emplace_back(transformed_point);
    }
    previous_pose = next_T;
  }
}
