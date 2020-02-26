#include <gtest/gtest.h>

#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>

// ia include solver stuff (instances)
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/solver.h"
// ia include types stuff (instances)
#include "srrg_solver/variables_and_factors/types_3d/instances.h"

const std::string exe_name = "test_se3_motion_based_calib";
#define LOG std::cerr << exe_name + "|"

using namespace srrg2_core;
using namespace srrg2_solver;

// ia global data
const size_t n_meas       = 100;
const size_t n_iterations = 20;

/* This function generates a fake relative 3d isometry */
Isometry3f randomRelativeIso() {
  const float tmax = 0.1;
  float x          = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / tmax));
  float y          = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / tmax));
  float z          = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / tmax));
  float rx         = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / tmax));
  float ry         = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / tmax));
  float rz         = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / tmax));
  Vector6f rand_pose;
  rand_pose << x, y, z, rx, ry, rz;
  Isometry3f rand_iso = srrg2_core::geometry3d::v2t(rand_pose);

  return rand_iso;
}

int main(int argc, char** argv) {
  registerTypes3D();
  solver_registerTypes();
  // linear_solver_registerTypes();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(DUMMY_DATA, SE3PoseMotionErrorFactorAD) {
  using VariableType    = VariableSE3QuaternionRightAD;
  using VariablePtrType = std::shared_ptr<VariableType>;
  using FactorType      = SE3PoseMotionErrorFactorDataDriven;
  using FactorPtrType   = std::shared_ptr<FactorType>;
  using IsometryVector  = std::vector<Isometry3f, Eigen::aligned_allocator<Isometry3f>>;
  // ia generate dummy data
  const Vector6f minimal_T = 10 * Vector6f::Random();
  const Isometry3f T       = geometry3d::ta2t(minimal_T);
  const Isometry3f inv_T   = T.inverse();

  IsometryVector measurements, relative_motions;
  CorrespondenceVector correspondences;

  measurements.reserve(n_meas);
  relative_motions.reserve(n_meas);
  correspondences.reserve(n_meas);
  // bdc generating dataset
  for (size_t m = 0; m < n_meas; ++m) {
    Isometry3f rel_sensor_motion = randomRelativeIso();
    relative_motions.emplace_back(rel_sensor_motion);
    Isometry3f measurement = inv_T * rel_sensor_motion * T;
    measurements.emplace_back(measurement);
    correspondences.emplace_back(m, m);
  }
  FactorPtrType factor = FactorPtrType(new FactorType);
  factor->setVariableId(0,0);
  factor->setFixed(measurements);
  factor->setMoving(relative_motions);
  factor->setCorrespondences(correspondences);

  VariablePtrType pose = VariablePtrType(new VariableType);
  pose->setEstimate(Isometry3f::Identity());
  pose->setGraphId(0);

  FactorGraphPtr graph(new FactorGraph);
  graph->addFactor(factor);
  graph->addVariable(pose);
  Solver solver;

  solver.param_max_iterations.pushBack(n_iterations);
  solver.param_termination_criteria.setValue(nullptr);
  solver.setGraph(graph);
  solver.compute();
  const auto& stats      = solver.iterationStats();
  const auto& final_chi2 = stats.back().chi_inliers;

  // ia assert performed iterations are the effectively n_iterations
  ASSERT_EQ(stats.size(), n_iterations);
  // ia assert chi2 is good
  ASSERT_LT(final_chi2, 1e-6);
  // ia assert that relative error is good
  const auto& estimated_T = pose->estimate();
  const auto diff_T       = estimated_T.inverse() * T;
  const auto diff_vector  = geometry3d::t2tnq(diff_T);
  ASSERT_LT(diff_vector[0], 1e-5); // ia X
  ASSERT_LT(diff_vector[1], 1e-5); // ia Y
  ASSERT_LT(diff_vector[2], 1e-5); // ia Z
  ASSERT_LT(diff_vector[3], 1e-5); // ia qx
  ASSERT_LT(diff_vector[4], 1e-5); // ia qy
  ASSERT_LT(diff_vector[5], 1e-5); // ia qz
  LOG << stats << std::endl;
}
