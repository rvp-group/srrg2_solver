#include <gtest/gtest.h>

#include "srrg2_solver_experiments/utility/stanford_bunny_converter.hpp"

#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/internals/linear_solvers/instances.h"
#include "srrg_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholesky.h"
#include "srrg_solver/solver_core/iteration_algorithm_lm.h"
#include "srrg_solver/variables_and_factors/types_2d/instances.h"
#include "srrg_solver/variables_and_factors/types_3d/instances.h"
#include "srrg_solver/variables_and_factors/types_3d/se3_point2point_error_factor.h"
#include "srrg_solver_extras/types_3d_ad/se3_point2point_euler_error_factor_ad.h"

#include <srrg_solver/solver_core/solver.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_test/test_helper.hpp>

#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/icp.h>

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace srrg2_solver_extras;
using namespace srrg2_solver_experiments;

const char* banner[] = {
  "This program takes a .ply data stanford bunny reconstructed instance and performs experiments",
  0};

static std::string filename   = "";
static std::string output_dir = "";

Point3fVectorCloud fixed_cloud;
Point3fVectorCloud moving_cloud;
CorrespondenceVector correspondences;
size_t n_iterations = 0;
float lambda        = 0.0f;
Isometry3f T;

int main(int argc, char** argv) {
  registerTypes2D();
  registerTypes3D();
  solver_registerTypes();
  linear_solver_registerTypes();

  srrgInit(argc, argv);
  ParseCommandLine command_line_parser(argv, banner);
  ArgumentString argument_input_file(
    &command_line_parser, "i", "input", "file path for .ply file", "");
  ArgumentString argument_output_directory(
    &command_line_parser, "od", "output-directory", "Results directory", "");
  ArgumentInt argument_iterations(
    &command_line_parser, "it", "iterations", "number of iterations", 10);
  ArgumentDouble argument_lambda(&command_line_parser, "l", "lambda", "lambda for LM", 1e-7);

  command_line_parser.parse();

  if (argument_input_file.value().empty()) {
    std::cerr << "ERROR: input file .ply is empty (set with -i)" << std::endl;
    return 0;
  }
  if (argument_output_directory.value().empty()) {
    std::cerr << "ERROR: out dir is empty (set with -od)" << std::endl;
    return 0;
  }

  filename   = argument_input_file.value();
  output_dir = argument_output_directory.value();
  if (output_dir.back() != '/') {
    output_dir.push_back('/');
  }
  n_iterations = argument_iterations.value();
  std::cerr << "n iterations: " << n_iterations << std::endl;
  lambda = argument_lambda.value();
  std::cerr << "lambda: " << lambda << std::endl;

  if (!srrg2_core::isAccessible(filename)) {
    std::cerr << "ERROR: " << filename << " is inaccessible" << std::endl;
    return 0;
  }
  if (!srrg2_core::isAccessible(output_dir)) {
    std::cerr << "ERROR: " << output_dir << " is inaccessible" << std::endl;
    return 0;
  }

  StanfordBunnyConverter conv(filename);
  conv.parsePLYFile();

  fixed_cloud = conv.pointcloud();

  moving_cloud = fixed_cloud;
  correspondences.reserve(fixed_cloud.size());

  for (size_t i = 0; i < fixed_cloud.size(); ++i) {
    correspondences.emplace_back(Correspondence(i, i));
  }

  //  srand((unsigned int) time(0));
  const Vector3f t            = Vector3f::Random() * 10;
  const Vector3f euler_angles = Vector3f::Random() * M_PI;
  T.setIdentity();
  T.linear()      = geometry3d::a2r(euler_angles);
  T.translation() = t;

  std::cerr << FG_YELLOW(T.matrix()) << std::endl;

  moving_cloud.transformInPlace<Isometry>(T);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(DUMMY_DATA, SE3Point2PointErrorFactor) {
  std::string filename(::testing::UnitTest::GetInstance()->current_test_info()->name());
  std::string filename_ate = output_dir + filename + "_ate.txt";
  filename += ".txt";
  filename = output_dir + filename;

  using VariableType    = VariableSE3QuaternionRight;
  using VariablePtrType = std::shared_ptr<VariableType>;
  using FactorType      = SE3Point2PointErrorFactorCorrespondenceDriven;
  using FactorPtrType   = std::shared_ptr<FactorType>;

  // create factor for icp
  FactorPtrType factor(new FactorType);
  factor->setVariableId(0, 0);

  // setup the factor;
  factor->setFixed(fixed_cloud);
  factor->setMoving(moving_cloud);
  factor->setCorrespondences(correspondences);
  factor->setInformationMatrix(Matrix3f::Identity());

  // declare a multi_solver
  Solver solver;
  solver.param_max_iterations.pushBack(n_iterations);
  solver.param_termination_criteria.setValue(nullptr);
  std::shared_ptr<IterationAlgorithmLM> lm(new IterationAlgorithmLM);
  lm->param_user_lambda_init.setValue(lambda);
  solver.param_algorithm.setValue(lm);

  // declare a graph,
  FactorGraphPtr graph(new FactorGraph);

  // create a variable, set an id and add it to the graph
  VariablePtrType pose(new VariableType);
  pose->setGraphId(0);
  graph->addVariable(pose);

  // add factor
  graph->addFactor(factor);
  graph->bindFactors();
  solver.setGraph(graph);

  ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));
  pose->setEstimate(Isometry3f::Identity());
  std::cerr << "computing... ";
  solver.compute();
  std::cerr << "done" << std::endl;

  const auto& stats      = solver.iterationStats();
  const auto& final_chi2 = stats.back().chi_inliers;
  // ia assert performed iterations are the effectively n_iterations
  ASSERT_EQ(stats.size(), n_iterations);
  // ia assert chi2 is good
  ASSERT_LT(final_chi2, 1e-6);
  // ia assert that relative error is good

  const auto& estimated_T    = pose->estimate().inverse();
  const Isometry3f diff_T    = estimated_T.inverse() * T;
  const Vector6f diff_vector = geometry3d::t2v(diff_T);

  std::cerr << estimated_T.matrix() << std::endl;

  std::ofstream out(filename);
  for (const IterationStats& stat : stats) {
    out << stat << std::endl;
  }
  //  out << stats;
  out.close();

  out.open(filename_ate);
  for (size_t i = 0; i < 6; ++i) {
    out << diff_vector[i] << "\t";
  }
  out << std::endl;
  out << diff_vector.head<3>().norm() << "\t" << diff_vector.tail<3>().norm();
  out.close();

  ASSERT_NEAR_EIGEN(diff_vector.head<3>(), Vector3f::Zero(), 1e-5);
  for (int i = 3; i < 6; ++i) {
    ASSERT_LT(std::sin(diff_vector[i]), 1e-5);
  }
}

TEST(DUMMY_DATA, SE3Point2PointErrorFactorAD) {
  std::string filename(::testing::UnitTest::GetInstance()->current_test_info()->name());
  std::string filename_ate = output_dir + filename + "_ate.txt";
  filename += ".txt";
  filename = output_dir + filename;

  using VariableType    = VariableSE3EulerRightAD;
  using VariablePtrType = std::shared_ptr<VariableType>;
  using FactorType      = SE3Point2PointEulerErrorFactorCorrespondenceDrivenAD;
  using FactorPtrType   = std::shared_ptr<FactorType>;

  // create factor for icp
  FactorPtrType factor(new FactorType);
  factor->setVariableId(0, 0);

  // setup the factor;
  factor->setFixed(fixed_cloud);
  factor->setMoving(moving_cloud);
  factor->setCorrespondences(correspondences);
  factor->setInformationMatrix(Matrix3f::Identity());

  // declare a multi_solver
  Solver solver;
  solver.param_max_iterations.pushBack(n_iterations);
  solver.param_termination_criteria.setValue(nullptr);
  std::shared_ptr<IterationAlgorithmLM> lm(new IterationAlgorithmLM);
  lm->param_user_lambda_init.setValue(lambda);
  solver.param_algorithm.setValue(lm);

  // declare a graph,
  FactorGraphPtr graph(new FactorGraph);

  // create a variable, set an id and add it to the graph
  VariablePtrType pose(new VariableType);
  pose->setGraphId(0);
  graph->addVariable(pose);

  // add factor
  graph->addFactor(factor);
  graph->bindFactors();
  solver.setGraph(graph);

  ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));
  pose->setEstimate(Isometry3f::Identity());
  std::cerr << "computing... ";
  solver.compute();
  std::cerr << "done" << std::endl;

  const auto& stats      = solver.iterationStats();
  const auto& final_chi2 = stats.back().chi_inliers;
  // ia assert performed iterations are the effectively n_iterations
  ASSERT_EQ(stats.size(), n_iterations);
  // ia assert chi2 is good
  ASSERT_LT(final_chi2, 1e-6);
  // ia assert that relative error is good

  const auto& estimated_T    = pose->estimate().inverse();
  const Isometry3f diff_T    = estimated_T.inverse() * T;
  const Vector6f diff_vector = geometry3d::t2ta(diff_T);

  std::cerr << estimated_T.matrix() << std::endl;

  std::ofstream out(filename);
  for (const IterationStats& stat : stats) {
    out << stat << std::endl;
  }
  //  out << stats;
  out.close();

  out.open(filename_ate);
  for (size_t i = 0; i < 6; ++i) {
    out << diff_vector[i] << "\t";
  }
  out << std::endl;
  out << diff_vector.head<3>().norm() << "\t" << diff_vector.tail<3>().norm();
  out.close();

  ASSERT_NEAR_EIGEN(diff_vector.head<3>(), Vector3f::Zero(), 1e-5);
  for (int i = 3; i < 6; ++i) {
    ASSERT_LT(std::sin(diff_vector[i]), 1e-5);
  }
}

using PCL3D    = pcl::PointCloud<pcl::PointXYZ>;
using PCL3DPtr = PCL3D::Ptr;
using namespace pcl;

class MyCE : public registration::CorrespondenceEstimation<PointXYZ, PointXYZ> {
public:
  typedef boost::shared_ptr<MyCE> Ptr;

  virtual void
  determineCorrespondences(pcl::Correspondences& correspondences_,
                           double max_distance = std::numeric_limits<double>::max()) override {
    correspondences_ = this->_correspondences;
  }

  pcl::Correspondences _correspondences;
};

void convertToPCL(PCL3DPtr out_cloud_, const Point3fVectorCloud& in_cloud_) {
  for (size_t i = 0; i < in_cloud_.size(); ++i) {
    const auto& p = in_cloud_[i];
    out_cloud_->push_back(
      pcl::PointXYZ(p.coordinates().x(), p.coordinates().y(), p.coordinates().z()));
  }
  out_cloud_->is_dense = true;
}

TEST(DUMMY_DATA, PCL) {
  std::string filename(::testing::UnitTest::GetInstance()->current_test_info()->name());
  std::string filename_ate = output_dir + filename + "_ate.txt";
  filename += ".txt";
  filename = output_dir + filename;

  std::cerr << "converting pcls" << std::endl;
  PCL3DPtr fixed(new PCL3D);
  PCL3DPtr moving(new PCL3D);

  convertToPCL(fixed, fixed_cloud);
  convertToPCL(moving, moving_cloud);
  pcl::Correspondences pcl_correspondences;
  pcl_correspondences.reserve(correspondences.size());

  for (const auto& c : correspondences) {
    pcl_correspondences.emplace_back(
      pcl::Correspondence(c.fixed_idx, c.moving_idx, std::numeric_limits<float>::max()));
  }

  using ICP = pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>;
  ICP icp;
  icp.setInputSource(fixed);
  icp.setInputTarget(moving);
  icp.setMaximumIterations(1);
  icp.setTransformationEpsilon(1e-6);

  MyCE::Ptr ce(new MyCE);
  ce->_correspondences = pcl_correspondences;
  icp.setCorrespondenceEstimation(ce);
  pcl::PointCloud<pcl::PointXYZ> Final;

  pcl::console::TicToc time;
  std::vector<double> alignment_time;
  alignment_time.reserve(n_iterations);
  std::vector<float> fitness;
  fitness.reserve(n_iterations);

  size_t iteration = 0;
  std::cerr << "computing... ";
  while (iteration++ < n_iterations) {
    SystemUsageCounter::tic();
    icp.align(Final, icp.getFinalTransformation());
    double toc = SystemUsageCounter::toc();
    alignment_time.emplace_back(toc);
    fitness.emplace_back(icp.getFitnessScore());
  }
  std::cerr << "done" << std::endl;

  size_t best_idx = fitness.size();
  //    std::distance(fitness.begin(), std::min_element(fitness.begin(), fitness.end()));

  std::stringstream ss;
  for (size_t i = 0; i < best_idx; ++i) {
    ss << "it= " << i << "; ";
    ss << "time_it= " << alignment_time[i] << "; ";
    ss << "fitness= " << fitness[i] << "; " << std::endl;
  }

  std::ofstream out(filename);
  out << ss.str();
  out.close();

  // ia assert fittness is good
  ASSERT_LT(icp.getFitnessScore(), 1e-6);

  std::cerr << icp.getFinalTransformation() << std::endl;

  const Isometry3f diff_T    = Isometry3f(icp.getFinalTransformation()).inverse() * T;
  const Vector6f diff_vector = geometry3d::t2v(diff_T);

  out.open(filename_ate);
  for (size_t i = 0; i < 6; ++i) {
    out << diff_vector[i] << "\t";
  }
  out << std::endl;
  out << diff_vector.head<3>().norm() << "\t" << diff_vector.tail<3>().norm();
  out.close();

  ASSERT_NEAR_EIGEN(diff_vector.head<3>(), Vector3f::Zero(), 1e-5);
  for (int i = 3; i < 6; ++i) {
    ASSERT_LT(std::sin(diff_vector[i]), 1e-5);
  }
}
#include <pcl/registration/transformation_estimation_lm.h>

TEST(DUMMY_DATA, PCL_LM) {
  std::string filename(::testing::UnitTest::GetInstance()->current_test_info()->name());
  std::string filename_ate = output_dir + filename + "_ate.txt";
  filename += ".txt";
  filename = output_dir + filename;

  std::cerr << "converting pcls" << std::endl;
  PCL3DPtr fixed(new PCL3D);
  PCL3DPtr moving(new PCL3D);

  convertToPCL(fixed, fixed_cloud);
  convertToPCL(moving, moving_cloud);
  pcl::Correspondences pcl_correspondences;
  pcl_correspondences.reserve(correspondences.size());

  for (const auto& c : correspondences) {
    pcl_correspondences.emplace_back(
      pcl::Correspondence(c.fixed_idx, c.moving_idx, std::numeric_limits<float>::max()));
  }

  using ICP = pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>;
  ICP icp;
  icp.setInputSource(fixed);
  icp.setInputTarget(moving);
  icp.setMaximumIterations(1);
  icp.setTransformationEpsilon(1e-6);

  MyCE::Ptr ce(new MyCE);
  ce->_correspondences = pcl_correspondences;
  icp.setCorrespondenceEstimation(ce);

  pcl::registration::TransformationEstimation<pcl::PointXYZ, pcl::PointXYZ, float>::Ptr
    te(new pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ, float>);
  icp.setTransformationEstimation(te);
  pcl::PointCloud<pcl::PointXYZ> Final;

  pcl::console::TicToc time;
  std::vector<double> alignment_time;
  alignment_time.reserve(n_iterations);
  std::vector<float> fitness;
  fitness.reserve(n_iterations);

  size_t iteration = 0;
  std::cerr << "computing...\n";
  while (iteration++ < n_iterations) {
    SystemUsageCounter::tic();
    icp.align(Final, icp.getFinalTransformation());
    double toc = SystemUsageCounter::toc();
    alignment_time.emplace_back(toc);
    fitness.emplace_back(icp.getFitnessScore());
  }
  std::cerr << "done" << std::endl;

  size_t best_idx = fitness.size();
  //    std::distance(fitness.begin(), std::min_element(fitness.begin(), fitness.end()));

  std::stringstream ss;
  for (size_t i = 0; i < best_idx; ++i) {
    ss << "it= " << i << "; ";
    ss << "time_it= " << alignment_time[i] << "; ";
    ss << "fitness= " << fitness[i] << "; " << std::endl;
  }

  std::ofstream out(filename);
  out << ss.str();
  out.close();

  // ia assert fittness is good
  ASSERT_LT(icp.getFitnessScore(), 1e-6);

  std::cerr << icp.getFinalTransformation() << std::endl;

  const Isometry3f diff_T    = Isometry3f(icp.getFinalTransformation()).inverse() * T;
  const Vector6f diff_vector = geometry3d::t2v(diff_T);

  out.open(filename_ate);
  for (size_t i = 0; i < 6; ++i) {
    out << diff_vector[i] << "\t";
  }
  out << std::endl;
  out << diff_vector.head<3>().norm() << "\t" << diff_vector.tail<3>().norm();
  out.close();

  ASSERT_NEAR_EIGEN(diff_vector.head<3>(), Vector3f::Zero(), 1e-4);
  for (int i = 3; i < 6; ++i) {
    ASSERT_LT(std::sin(diff_vector[i]), 1e-4);
  }
}


