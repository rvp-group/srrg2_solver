#include <random>
#include <vector>

#include <gtest/gtest.h>

#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>

// ia include solver stuff (instances)
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/iteration_algorithm_lm.h"
#include "srrg_solver/solver_core/solver.h"
// ia include types stuff (instances)
#include "srrg_solver/variables_and_factors/types_3d/instances.h"

const std::string exe_name = "test_se3_matchables";
#define LOG std::cerr << exe_name + "|"
const std::string data_folder(SRRG2_SOLVER_DATA_FOLDER);

using namespace srrg2_core;
using namespace srrg2_solver;

// ia global data
const size_t n_points = 10;
const size_t n_lines  = 10;
const size_t n_planes = 10;

const size_t n_iterations = 50;

int main(int argc, char** argv) {
  registerTypes3D();
  solver_registerTypes();
  // linear_solver_registerTypes();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// tg Vinia is happy 
TEST(EXAMPLE_GRAPH, SE3PoseMatchableEulerLeftErrorFactor) {
  // ia test data, contains no noise in the edges, but every node has been zeroed previously
  const std::string input_file = data_folder + "/matchable_graph.boss";
  LOG << "Try to read graph" << std::endl;
  FactorGraphPtr graph = FactorGraph::read(input_file);
  ASSERT_TRUE(graph != nullptr);
  LOG << "Graph set" << std::endl;
  Solver solver;
  std::shared_ptr<IterationAlgorithmGN> gn =
    std::dynamic_pointer_cast<IterationAlgorithmGN>(solver.param_algorithm.value());
  gn->param_damping.setValue(0.0001);
  LOG << "Gn set" << std::endl;
  solver.param_termination_criteria.setValue(nullptr);
  solver.param_max_iterations.pushBack(n_iterations);
  LOG << "Params set" << std::endl;
  solver.setGraph(graph);
  solver.compute();
  LOG << "Compute Done " << std::endl;
  const auto& stats = solver.iterationStats();
  LOG << "stats:\n\n" << stats << std::endl;
  const auto& final_chi2 = stats.back().chi_inliers;

  ASSERT_LT(final_chi2, 8e-5);
}

TEST(DUMMY_DATA, SE3Matchable2MatchableEulerLeftErrorFactor) {
  const size_t n_matchables = n_points + n_lines + n_planes;

  const Vector6f minimal_T = Vector6f::Random();
  const Isometry3f T       = geometry3d::ta2t(minimal_T);
  // const Isometry3f inv_T   = T.inverse();

  // ia construct two identic clouds
  MatchablefVector cloud_moving; // ia the map
  MatchablefVector cloud_fixed;  // ia new frame

  cloud_fixed.reserve(n_matchables);
  for (size_t k = 0; k < n_points; ++k) {
    Matchablef p_m = Matchablef(MatchableBase::Type::Point, Vector3f::Random());
    cloud_fixed.emplace_back(p_m);
  }

  for (size_t k = 0; k < n_lines; ++k) {
    Matchablef l_m = Matchablef(MatchableBase::Type::Line, Vector3f::Random());
    l_m.setDirection(Vector3f::UnitY());
    cloud_fixed.emplace_back(l_m);
  }

  for (size_t k = 0; k < n_planes; ++k) {
    Matchablef p_m = Matchablef(MatchableBase::Type::Plane, Vector3f::Random());
    p_m.setDirection(Vector3f::UnitZ());
    cloud_fixed.emplace_back(p_m);
  }

  cloud_moving.resize(n_matchables);
  for (size_t k = 0; k < n_matchables; ++k) {
    cloud_moving[k] = cloud_fixed[k].transform(T);
  }
  // declare a solver
  Solver solver;

  // declare a graph,
  FactorGraphPtr graph(new FactorGraph);

  // create a variable, set an id and add it to the graph
  std::shared_ptr<VariableSE3EulerLeftAD> pose(new VariableSE3EulerLeftAD);
  pose->setGraphId(0);
  graph->addVariable(pose);

  // create an ICP factor, correspondence driven, set the var index, and add it to the graph
  std::shared_ptr<SE3Matchable2MatchableEulerLeftErrorFactorCorrespondenceDriven> factor(
    new SE3Matchable2MatchableEulerLeftErrorFactorCorrespondenceDriven);
  factor->setVariableId(0, 0);
  graph->addFactor(factor);

  solver.setGraph(graph);

  CorrespondenceVector correspondences;
  for (size_t i = 0; i < n_matchables; ++i) {
    correspondences.push_back(Correspondence(i, i));
  }
  factor->setFixed(cloud_fixed);
  factor->setMoving(cloud_moving);
  factor->setCorrespondences(correspondences);
  solver.param_max_iterations.pushBack(n_iterations);
  solver.param_termination_criteria.setValue(nullptr);
  ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));

  // ia set initial guess and compute
  pose->setEstimate(Isometry3f::Identity());
  solver.compute();
  const auto& stats      = solver.iterationStats();
  const auto& final_chi2 = stats.back().chi_inliers;

  // ia assert performed iterations are the effectively n_iterations
  ASSERT_EQ(stats.size(), n_iterations);
  // ia assert chi2 is good
  ASSERT_LT(final_chi2, 1e-6);

  LOG << stats << std::endl;
  // ia assert that relative error is good
  const auto& estimated_T = pose->estimate();

  const auto diff_T      = estimated_T.inverse() * T;
  const auto diff_vector = geometry3d::t2tnq(diff_T);
  ASSERT_LT(diff_vector[0], 1e-5); // ia X
  ASSERT_LT(diff_vector[1], 1e-5); // ia Y
  ASSERT_LT(diff_vector[2], 1e-5); // ia Z
  ASSERT_LT(diff_vector[3], 1e-5); // ia qx
  ASSERT_LT(diff_vector[4], 1e-5); // ia qy
  ASSERT_LT(diff_vector[5], 1e-5); // ia qz
}
