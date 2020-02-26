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
