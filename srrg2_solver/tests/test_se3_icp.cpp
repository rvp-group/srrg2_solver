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
  registerTypes3D();
  solver_registerTypes();
  // linear_solver_registerTypes();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(DUMMY_DATA, SE3Point2PointErrorFactor) {
  using VariableType       = VariableSE3QuaternionRight;
  using VariablePtrType    = std::shared_ptr<VariableType>;
  using FactorType         = SE3Point2PointErrorFactorCorrespondenceDriven;
  using FactorPtrType      = std::shared_ptr<FactorType>;
  const Vector3f t = 10 * Vector3f::Random();
  Quaternionf q(0.3,0.3,0.3,0.3);
  q.normalize();

  Isometry3f T;
  T.setIdentity();
  T.linear() = q.toRotationMatrix();
  T.translation() = t;
  const Isometry3f inv_T   = T.inverse();

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
    moving_point.coordinates() = inv_T * fixed_point.coordinates();
    fixed_cloud.emplace_back(fixed_point);
    moving_cloud.emplace_back(moving_point);
    correspondences.emplace_back(Correspondence(i, i));
  }

  // setup the factor;
  factor->setFixed(fixed_cloud);
  factor->setMoving(moving_cloud);
  factor->setCorrespondences(correspondences);
  factor->setInformationMatrix(Matrix3f::Identity());

  solver.param_max_iterations.pushBack(n_iterations);
  solver.param_termination_criteria.setValue(nullptr);
  ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));

  pose->setEstimate(Isometry3f::Identity());
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
  LOG << stats << std::endl;

  ASSERT_LT(diff_vector.x(), 1e-5);
  ASSERT_LT(diff_vector.y(), 1e-5);
  ASSERT_LT(diff_vector.z(), 1e-5);
}
