#include <random>
#include <srrg_data_structures/correspondence.h>
#include <srrg_test/synthetic_world.hpp>

#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/variables_and_factors/types_3d/instances.h"
#include "srrg_solver/variables_and_factors/types_projective/instances.h"

using namespace srrg2_core;
using namespace srrg2_solver;

// ds TODO add proper noise on projections
// ds TODO add unittest for mean disparity weighted jacs

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_);
}

// ds projective monocular and rectified stereo test fixture
class Pinhole : public ::testing::Test {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void SetUp() override {
    solver_registerTypes();
    // linear_solver_registerTypes();
    registerTypes3D();
    projective_registerTypes();
    std::srand(0);

    // ds setup solver ..
    ASSERT_NOTNULL(solver.param_algorithm.value());
    ASSERT_NOTNULL(solver.param_linear_solver.value());
    solver.param_termination_criteria.setValue(nullptr);
    solver.param_max_iterations.pushBack(maximum_number_of_iterations);

    // ds .. and robustifiers
    robustifier_saturated.param_chi_threshold.setValue(25 * 25);

    // ds setup graph
    graph = FactorGraphPtr(new FactorGraph());

    // create a variable, set an id and add it to the graph
    X = std::shared_ptr<VariableSE3QuaternionRightAD>(new VariableSE3QuaternionRightAD());
    X->setGraphId(0);
    graph->addVariable(X);
    solver.setGraph(graph);

    // ds set target transform between the two states
    moving_in_fixed.setIdentity();

    // ds walking backwards
    moving_in_fixed.translation() = Vector3f(1, 2, 3);

    // ds rotate 18 degrees around principal camera axis
    moving_in_fixed.rotate(AngleAxisf(0.1 * M_PI, Vector3f::UnitZ()));

    // ds set camera matrix
    camera_calibration_matrix << 500, 0, 400, 0, 500, 200, 0, 0, 1;

    // ds compute baseline to right camera (parallel optical axis)
    baseline_pixels = camera_calibration_matrix * baseline_meters;
  }

  void TearDown() override {
    // ds no dynamic memory
  }

  void generateProblemPnP(const size_t& number_of_measurements_,
                          const float noise_magnitude_      = 0,
                          const size_t& number_of_outliers_ = 0) {
    // ds noise generation that is applied to measurements (fixed)
    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(0.0f, noise_magnitude_ * 1.0);

    // srrg add sensor's offset
    Isometry3f camera_pose = moving_in_fixed * sensor_in_robot;

    // ds generate points and measurements until we have the target number
    uint32_t index_point     = 0;
    size_t outlier_frequency = number_of_measurements_ + 1; // ds default no outliers
    if (number_of_outliers_ > 0) {
      outlier_frequency = number_of_measurements_ / number_of_outliers_;
    }
    while (index_point < number_of_measurements_) {
      // ds obtain a 3D point in the camera frame with positive depth
      Point3f moving_point_in_camera_previous;
      moving_point_in_camera_previous.coordinates() = Vector3f::Random() * 100.0f;
      moving_point_in_camera_previous.coordinates().z() =
        std::fabs(moving_point_in_camera_previous.coordinates().z());
      const Vector3f point_in_camera(camera_pose * moving_point_in_camera_previous.coordinates());

      // ds project the 3D point into the left and right camera image planes
      const Vector3f coordinates_homogeneous_left  = camera_calibration_matrix * point_in_camera;
      const Vector3f coordinates_homogeneous_right = coordinates_homogeneous_left + baseline_pixels;

      // ds obtain image coordinates of left and right projection
      Point4f fixed_point;
      fixed_point.coordinates()(0) =
        coordinates_homogeneous_left.x() / coordinates_homogeneous_left.z();
      fixed_point.coordinates()(1) =
        coordinates_homogeneous_left.y() / coordinates_homogeneous_left.z();
      fixed_point.coordinates()(2) =
        coordinates_homogeneous_right.x() / coordinates_homogeneous_right.z();
      fixed_point.coordinates()(3) =
        coordinates_homogeneous_right.y() / coordinates_homogeneous_right.z();
      assert(fixed_point.coordinates()(1) == fixed_point.coordinates()(3) &&
             "assuming rectified stereo");

      // ds add noise to feature position measurements
      fixed_point.coordinates()(0) += distribution(generator);
      fixed_point.coordinates()(1) += distribution(generator);
      fixed_point.coordinates()(2) += distribution(generator);

      // ds set the same noise to v coordinates to mimic a rectified rigid stereo scenario)
      fixed_point.coordinates()(3) = fixed_point.coordinates()(1);

      // ds check if the point is out of the field of view (i.e. not measurable) - keep sampling
      if (fixed_point.coordinates()(0) < 0 || fixed_point.coordinates()(0) > image_dimension.x() ||
          fixed_point.coordinates()(1) < 0 || fixed_point.coordinates()(1) > image_dimension.y() ||
          fixed_point.coordinates()(2) < 0 || fixed_point.coordinates()(2) > image_dimension.x() ||
          fixed_point.coordinates()(3) < 0 || fixed_point.coordinates()(3) > image_dimension.y()) {
        continue;
      }

      // ds monocular view
      Point2f fixed_point_left;
      fixed_point_left.coordinates()(0) = fixed_point.coordinates()(0);
      fixed_point_left.coordinates()(1) = fixed_point.coordinates()(1);

      // ds monocular view with depth
      Point3f fixed_point_left_with_depth;
      fixed_point_left_with_depth.coordinates()(0) = fixed_point_left.coordinates()(0);
      fixed_point_left_with_depth.coordinates()(1) = fixed_point_left.coordinates()(1);
      fixed_point_left_with_depth.coordinates()(2) =
        point_in_camera(2) + distribution(generator) / 100.0; // ds fraction of pixel noise on depth

      // ds store generated point and its projections with correspondence information
      points_fixed_mono.push_back(fixed_point_left);
      points_fixed_mono_depth.push_back(fixed_point_left_with_depth);
      points_fixed_stereo.push_back(fixed_point);
      points_moving.push_back(moving_point_in_camera_previous);

      // ds check if we have to generate an outlier (including duplicate associations)
      if ((index_point + 1) % outlier_frequency == 0) {
        const size_t index_invalid = std::rand() % number_of_measurements_;
        correspondences.push_back(Correspondence(index_point, index_invalid));
      } else {
        correspondences.push_back(Correspondence(index_point, index_point));
      }
      ++index_point;
    }
    //    points_moving.transformInPlace<Isometry>(sensor_in_robot);
  }

protected:
  // ds variable to optimize
  std::shared_ptr<VariableSE3QuaternionRightAD> X;

  // ds solver used in all tests
  Solver solver;

  // ds maximum number of iterations
  static constexpr size_t maximum_number_of_iterations = 25;

  // ds graph handle
  FactorGraphPtr graph = nullptr;

  // ds optional robustifier that can be hooked to factors
  RobustifierSaturated robustifier_saturated;

  // ds sampling configuration
  Isometry3f moving_in_fixed = Isometry3f::Identity();

  // ds image dimension (rows, cols) - used for world generation
  Vector2f image_dimension = Vector2f(1000, 1000);

  // ds pinhole camera matrix
  Matrix3f camera_calibration_matrix;

  // ds base line in physical and camera
  Vector3f baseline_meters = Vector3f(-1, 0, 0);
  Vector3f baseline_pixels;

  // ds point clouds, information and correspondences
  Point2fVectorCloud points_fixed_mono;
  Point3fVectorCloud points_fixed_mono_depth;
  Point4fVectorCloud points_fixed_stereo;
  Point3fVectorCloud points_moving;
  CorrespondenceVector correspondences;

  // srrg sensor pose wrt base frame
  Isometry3f sensor_in_robot = Isometry3f::Identity();
};

TEST_F(Pinhole, SE3ProjectiveErrorFactorCorrespondenceDriven_NastyConfiguration) {
  // ds move world origin
  moving_in_fixed.linear() = geometry3d::a2r(Vector3f(M_PI / 4, 0, 0));
  std::cerr << "origin transform: \n" << moving_in_fixed.matrix() << std::endl;
  std::cerr << "origin euler: " << geometry3d::t2ta(moving_in_fixed).transpose() << std::endl;
  std::cerr << "origin quaternion: " << geometry3d::t2tnq(moving_in_fixed).transpose() << std::endl;

  // ds generate world
  generateProblemPnP(1000 /*number of measurements*/, 0 /*noise magnitude*/, 0 /*# outliers*/);

  // ds create a stereo posit factor
  std::shared_ptr<SE3ProjectiveErrorFactorCorrespondenceDriven> factor(
    new SE3ProjectiveErrorFactorCorrespondenceDriven());
  factor->setVariableId(0, 0);

  // setup the factor
  factor->setEnabled(true);
  factor->setCameraMatrix(camera_calibration_matrix);
  factor->setImageDim(image_dimension);
  factor->setFixed(points_fixed_mono);
  factor->setMoving(points_moving);
  factor->setCorrespondences(correspondences);
  Matrix2f information_matrix(Matrix2f::Identity());
  factor->setInformationMatrix(information_matrix);
  ASSERT_EQ(factor->size(), correspondences.size());

  // ds hook up factor, graph and solver
  graph->addFactor(factor);
  graph->bindFactors();
  ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));

  // ds set initial guess and optimize!
  Isometry3f initial_guess(Isometry3f::Identity());
  initial_guess.linear() = moving_in_fixed.linear(); // ds be gentle in orientation
  X->setEstimate(initial_guess);
  solver.compute();

  // ds validate optimization
  const auto& stats = solver.iterationStats();
  std::cerr << stats;
  ASSERT_EQ(stats.size(), static_cast<size_t>(maximum_number_of_iterations));
  ASSERT_LT(stats.back().chi_inliers, 2 /*ayyyy*/);

  // ds evaluate estimation error
  const Vector6f relative_error = geometry3d::t2tnq(X->estimate().inverse() * moving_in_fixed);
  ASSERT_LT(relative_error.norm(), 0.01 /*buaaack*/);
}

TEST_F(Pinhole, SE3ProjectiveErrorFactorCorrespondenceDriven) {
  generateProblemPnP(1000 /*number of measurements*/, 0 /*noise magnitude*/, 0 /*# outliers*/);

  // ds create a stereo posit factor
  std::shared_ptr<SE3ProjectiveErrorFactorCorrespondenceDriven> factor(
    new SE3ProjectiveErrorFactorCorrespondenceDriven());
  factor->setVariableId(0, 0);

  // setup the factor
  factor->setEnabled(true);
  factor->setCameraMatrix(camera_calibration_matrix);
  factor->setImageDim(image_dimension);
  factor->setFixed(points_fixed_mono);
  factor->setMoving(points_moving);
  factor->setCorrespondences(correspondences);
  Matrix2f information_matrix(Matrix2f::Identity());
  factor->setInformationMatrix(information_matrix);
  ASSERT_EQ(factor->size(), correspondences.size());

  // ds hook up factor, graph and solver
  graph->addFactor(factor);
  graph->bindFactors();
  ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));

  // ds set initial guess and optimize!
  X->setEstimate(Isometry3f::Identity());
  solver.compute();

  // ds validate optimization
  const auto& stats = solver.iterationStats();
  std::cerr << stats;
  ASSERT_EQ(stats.size(), static_cast<size_t>(maximum_number_of_iterations));
  ASSERT_LT(stats.back().chi_inliers, 1e-5);

  // ds evaluate estimation error
  const Vector6f relative_error = geometry3d::t2tnq(X->estimate().inverse() * moving_in_fixed);
  ASSERT_LT(relative_error.norm(), 1e-5);
}

TEST_F(Pinhole, SE3ProjectiveWithSensorErrorFactorCorrespondenceDriven) {
  sensor_in_robot.translation() = Vector3f(0.2, 0.3, 0.4);
  sensor_in_robot.linear()      = geometry3d::a2r(Vector3f(0, M_PI * 0.05, 0));

  generateProblemPnP(1000 /*number of measurements*/, 0 /*noise magnitude*/, 0 /*# outliers*/);
  // ds create a stereo posit factor
  std::shared_ptr<SE3ProjectiveWithSensorErrorFactorCorrespondenceDriven> factor(
    new SE3ProjectiveWithSensorErrorFactorCorrespondenceDriven());
  factor->setVariableId(0, 0);

  // setup the factor
  factor->setEnabled(true);
  factor->setCameraMatrix(camera_calibration_matrix);
  factor->setImageDim(image_dimension);
  factor->setFixed(points_fixed_mono);
  factor->setMoving(points_moving);
  factor->setCorrespondences(correspondences);
  factor->setSensorInRobot(sensor_in_robot);
  Matrix2f information_matrix(Matrix2f::Identity());
  factor->setInformationMatrix(information_matrix);
  ASSERT_EQ(factor->size(), correspondences.size());

  // ds hook up factor, graph and solver
  graph->addFactor(factor);
  graph->bindFactors();
  ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));

  // ds set initial guess and optimize!
  X->setEstimate(Isometry3f::Identity());
  solver.compute();

  // ds validate optimization
  const auto& stats = solver.iterationStats();
  std::cerr << stats;
  ASSERT_EQ(stats.size(), static_cast<size_t>(maximum_number_of_iterations));
  ASSERT_LT(stats.back().chi_inliers, 1e-5);

  // ds evaluate estimation error
  const Vector6f relative_error = geometry3d::t2tnq(X->estimate().inverse() * moving_in_fixed);
  ASSERT_LT(relative_error.norm(), 1e-5);
}

TEST_F(Pinhole, SE3ProjectiveDepthErrorFactorCorrespondenceDriven) {
  generateProblemPnP(1000 /*number of measurements*/, 0 /*noise magnitude*/, 0 /*# outliers*/);

  // ds create a stereo posit factor
  std::shared_ptr<SE3ProjectiveDepthErrorFactorCorrespondenceDriven> factor(
    new SE3ProjectiveDepthErrorFactorCorrespondenceDriven());
  factor->setVariableId(0, 0);

  // setup the factor
  factor->setEnabled(true);
  factor->setCameraMatrix(camera_calibration_matrix);
  factor->setImageDim(image_dimension);
  factor->setFixed(points_fixed_mono_depth);
  factor->setMoving(points_moving);
  factor->setCorrespondences(correspondences);
  Matrix3f information_matrix(Matrix3f::Identity());
  information_matrix(2, 2) = 10; // ds weight depth error higher (meters vs pixels)
  factor->setInformationMatrix(information_matrix);
  ASSERT_EQ(factor->size(), correspondences.size());

  // ds hook up factor, graph and solver
  graph->addFactor(factor);
  graph->bindFactors();
  ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));

  // ds set initial guess and optimize!
  X->setEstimate(Isometry3f::Identity());
  solver.compute();

  // ds validate optimization
  const auto& stats = solver.iterationStats();
  std::cerr << stats;
  ASSERT_EQ(stats.size(), static_cast<size_t>(maximum_number_of_iterations));
  ASSERT_LT(stats.back().chi_inliers, 1e-5);

  // ds evaluate estimation error
  const Vector6f relative_error = geometry3d::t2tnq(X->estimate().inverse() * moving_in_fixed);
  ASSERT_LT(relative_error.norm(), 1e-5);
}

// TEST_F(Pinhole, SE3ProjectiveDepthWithSensorErrorFactorCorrespondenceDriven) {
//   sensor_in_robot.translation() = Vector3f(0.2, 0.3, 0.4);
//   sensor_in_robot.linear()      = geometry3d::a2r(Vector3f(0, M_PI * 0.05, 0));
//
//   generateProblemPnP(1000 /*number of measurements*/, 0 /*noise magnitude*/, 0 /*# outliers*/);
//
//   // ds create a stereo posit factor
//   std::shared_ptr<SE3ProjectiveDepthWithSensorErrorFactorCorrespondenceDriven> factor(
//     new SE3ProjectiveDepthWithSensorErrorFactorCorrespondenceDriven());
//   factor->setVariableId(0, 0);
//
//   // setup the factor
//   factor->setEnabled(true);
//   factor->setCameraMatrix(camera_calibration_matrix);
//   factor->setImageDim(image_dimension);
//   factor->setFixed(points_fixed_mono_depth);
//   factor->setMoving(points_moving);
//   factor->setCorrespondences(correspondences);
//   factor->setSensorInRobot(sensor_in_robot);
//   Matrix3f information_matrix(Matrix3f::Identity());
//   information_matrix(2, 2) = 10; // ds weight depth error higher (meters vs pixels)
//   factor->setInformationMatrix(information_matrix);
//   ASSERT_EQ(factor->size(), correspondences.size());
//
//   // ds hook up factor, graph and solver
//   graph->addFactor(factor);
//   graph->bindFactors();
//   ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));
//
//   // ds set initial guess and optimize!
//   X->setEstimate(Isometry3f::Identity());
//   solver.compute();
//
//   // ds validate optimization
//   const auto& stats = solver.iterationStats();
//   std::cerr << stats;
//   ASSERT_EQ(stats.size(), static_cast<size_t>(maximum_number_of_iterations));
//   ASSERT_LT(stats.back().chi_inliers, 1e-5);
//
//   // ds evaluate estimation error
//   const Vector6f relative_error = geometry3d::t2tnq(X->estimate().inverse() * moving_in_fixed);
//   ASSERT_LT(relative_error.norm(), 1e-5);
// }

TEST_F(Pinhole, SE3RectifiedStereoProjectiveErrorFactorCorrespondenceDriven) {
  generateProblemPnP(1000 /*number of measurements*/, 0 /*noise magnitude*/, 0 /*# outliers*/);

  // ds create a stereo posit factor
  std::shared_ptr<SE3RectifiedStereoProjectiveErrorFactorCorrespondenceDriven> factor(
    new SE3RectifiedStereoProjectiveErrorFactorCorrespondenceDriven());
  factor->setVariableId(0, 0);

  // setup the factor
  factor->setEnabled(true);
  factor->setCameraMatrix(camera_calibration_matrix);
  factor->setImageDim(image_dimension);
  factor->setBaselinePixels(baseline_pixels);
  factor->setFixed(points_fixed_stereo);
  factor->setMoving(points_moving);
  factor->setCorrespondences(correspondences);
  Matrix3f information_matrix(Matrix3f::Identity());
  information_matrix(1, 1) = 2; // ds weight error in v twice (rectified)
  factor->setInformationMatrix(information_matrix);
  ASSERT_EQ(factor->size(), correspondences.size());

  // ds hook up factor, graph and solver
  graph->addFactor(factor);
  graph->bindFactors();
  ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));

  // ds set initial guess and optimize!
  X->setEstimate(Isometry3f::Identity());
  solver.compute();

  // ds validate optimization
  const auto& stats = solver.iterationStats();
  std::cerr << stats;
  ASSERT_EQ(stats.size(), static_cast<size_t>(maximum_number_of_iterations));
  ASSERT_LT(stats.back().chi_inliers, 1e-5);

  // ds evaluate estimation error
  const Vector6f relative_error = geometry3d::t2tnq(X->estimate().inverse() * moving_in_fixed);
  ASSERT_LT(relative_error.norm(), 1e-5);
}

TEST_F(Pinhole, SE3RectifiedStereoProjectiveWithSensorErrorFactorCorrespondenceDriven) {
  sensor_in_robot.translation() = Vector3f(0.2, 0.3, 0.4);
  sensor_in_robot.linear()      = geometry3d::a2r(Vector3f(0, M_PI * 0.05, 0));

  generateProblemPnP(1000 /*number of measurements*/, 0 /*noise magnitude*/, 0 /*# outliers*/);

  // ds create a stereo posit factor
  std::shared_ptr<SE3RectifiedStereoProjectiveWithSensorErrorFactorCorrespondenceDriven> factor(
    new SE3RectifiedStereoProjectiveWithSensorErrorFactorCorrespondenceDriven());
  factor->setVariableId(0, 0);

  // setup the factor
  factor->setEnabled(true);
  factor->setCameraMatrix(camera_calibration_matrix);
  factor->setImageDim(image_dimension);
  factor->setBaselinePixels(baseline_pixels);
  factor->setFixed(points_fixed_stereo);
  factor->setMoving(points_moving);
  factor->setCorrespondences(correspondences);
  factor->setSensorInRobot(sensor_in_robot);
  Matrix3f information_matrix(Matrix3f::Identity());
  information_matrix(1, 1) = 2; // ds weight error in v twice (rectified)
  factor->setInformationMatrix(information_matrix);
  ASSERT_EQ(factor->size(), correspondences.size());

  // ds hook up factor, graph and solver
  graph->addFactor(factor);
  graph->bindFactors();
  ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));

  // ds set initial guess and optimize!
  X->setEstimate(Isometry3f::Identity());
  solver.compute();

  // ds validate optimization
  const auto& stats = solver.iterationStats();
  std::cerr << stats;
  ASSERT_EQ(stats.size(), static_cast<size_t>(maximum_number_of_iterations));
  ASSERT_LT(stats.back().chi_inliers, 1e-5);

  // ds evaluate estimation error
  const Vector6f relative_error = geometry3d::t2tnq(X->estimate().inverse() * moving_in_fixed);
  ASSERT_LT(relative_error.norm(), 1e-5);
}

TEST_F(Pinhole, SE3ProjectiveErrorFactorCorrespondenceDriven_Noise) {
  generateProblemPnP(1000 /*number of measurements*/, 1 /*noise magnitude*/, 0 /*# outliers*/);

  // ds create a stereo posit factor
  std::shared_ptr<SE3ProjectiveErrorFactorCorrespondenceDriven> factor(
    new SE3ProjectiveErrorFactorCorrespondenceDriven());
  factor->setVariableId(0, 0);

  // setup the factor
  factor->setEnabled(true);
  factor->setCameraMatrix(camera_calibration_matrix);
  factor->setImageDim(image_dimension);
  factor->setFixed(points_fixed_mono);
  factor->setMoving(points_moving);
  factor->setCorrespondences(correspondences);
  Matrix2f information_matrix(Matrix2f::Identity());
  factor->setInformationMatrix(information_matrix);
  ASSERT_EQ(factor->size(), correspondences.size());

  // ds hook up factor, graph and solver
  graph->addFactor(factor);
  graph->bindFactors();
  ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));

  // ds set initial guess and optimize!
  X->setEstimate(Isometry3f::Identity());
  solver.compute();

  // ds validate optimization
  const auto& stats = solver.iterationStats();
  std::cerr << stats;
  ASSERT_EQ(stats.size(), static_cast<size_t>(maximum_number_of_iterations));
  ASSERT_LT(stats.back().chi_inliers, 250);

  // ds evaluate estimation error
  const Vector6f relative_error = geometry3d::t2tnq(X->estimate().inverse() * moving_in_fixed);
  ASSERT_LT(relative_error.norm(), 0.05);
}

TEST_F(Pinhole, SE3ProjectiveDepthErrorFactorCorrespondenceDriven_Noise) {
  generateProblemPnP(1000 /*number of measurements*/, 1 /*noise magnitude*/, 0 /*# outliers*/);

  // ds create a stereo posit factor
  std::shared_ptr<SE3ProjectiveDepthErrorFactorCorrespondenceDriven> factor(
    new SE3ProjectiveDepthErrorFactorCorrespondenceDriven());
  factor->setVariableId(0, 0);

  // setup the factor
  factor->setEnabled(true);
  factor->setCameraMatrix(camera_calibration_matrix);
  factor->setImageDim(image_dimension);
  factor->setFixed(points_fixed_mono_depth);
  factor->setMoving(points_moving);
  factor->setCorrespondences(correspondences);
  Matrix3f information_matrix(Matrix3f::Identity());
  information_matrix(2, 2) = 10; // ds weight depth error higher (meters vs pixels)
  factor->setInformationMatrix(information_matrix);
  ASSERT_EQ(factor->size(), correspondences.size());

  // ds hook up factor, graph and solver
  graph->addFactor(factor);
  graph->bindFactors();
  ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));

  // ds set initial guess and optimize!
  X->setEstimate(Isometry3f::Identity());
  solver.compute();

  // ds validate optimization
  const auto& stats = solver.iterationStats();
  std::cerr << stats;
  ASSERT_EQ(stats.size(), static_cast<size_t>(maximum_number_of_iterations));
  ASSERT_LT(stats.back().chi_inliers, 250);

  // ds evaluate estimation error
  const Vector6f relative_error = geometry3d::t2tnq(X->estimate().inverse() * moving_in_fixed);
  ASSERT_LT(relative_error.norm(), 0.05);
}

TEST_F(Pinhole, SE3RectifiedStereoProjectiveErrorFactorCorrespondenceDriven_Noise) {
  generateProblemPnP(1000 /*number of measurements*/, 1 /*noise magnitude*/, 0 /*# outliers*/);

  // ds create a stereo posit factor
  std::shared_ptr<SE3RectifiedStereoProjectiveErrorFactorCorrespondenceDriven> factor(
    new SE3RectifiedStereoProjectiveErrorFactorCorrespondenceDriven());
  factor->setVariableId(0, 0);

  // setup the factor
  factor->setEnabled(true);
  factor->setCameraMatrix(camera_calibration_matrix);
  factor->setImageDim(image_dimension);
  factor->setBaselinePixels(baseline_pixels);
  factor->setFixed(points_fixed_stereo);
  factor->setMoving(points_moving);
  factor->setCorrespondences(correspondences);
  Matrix3f information_matrix(Matrix3f::Identity());
  information_matrix(1, 1) = 2; // ds weight error in v twice (rectified)
  factor->setInformationMatrix(information_matrix);
  ASSERT_EQ(factor->size(), correspondences.size());

  // ds hook up factor, graph and solver
  graph->addFactor(factor);
  graph->bindFactors();
  ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));

  // ds set initial guess and optimize!
  X->setEstimate(Isometry3f::Identity());
  solver.compute();

  // ds validate optimization
  const auto& stats = solver.iterationStats();
  std::cerr << stats;
  ASSERT_EQ(stats.size(), static_cast<size_t>(maximum_number_of_iterations));
  ASSERT_LT(stats.back().chi_inliers, 500);

  // ds evaluate estimation error
  const Vector6f relative_error = geometry3d::t2tnq(X->estimate().inverse() * moving_in_fixed);
  ASSERT_LT(relative_error.norm(), 0.05);
}

TEST_F(Pinhole, SE3ProjectiveErrorFactorCorrespondenceDriven_Noise_Outliers) {
  generateProblemPnP(1000 /*number of measurements*/, 1 /*noise magnitude*/, 100 /*# outliers*/);

  // ds create a stereo posit factor
  std::shared_ptr<SE3ProjectiveErrorFactorCorrespondenceDriven> factor(
    new SE3ProjectiveErrorFactorCorrespondenceDriven());
  factor->setVariableId(0, 0);

  // setup the factor
  factor->setEnabled(true);
  factor->setCameraMatrix(camera_calibration_matrix);
  factor->setImageDim(image_dimension);
  factor->setFixed(points_fixed_mono);
  factor->setMoving(points_moving);
  factor->setCorrespondences(correspondences);
  Matrix2f information_matrix(Matrix2f::Identity());
  factor->setInformationMatrix(information_matrix);
  ASSERT_EQ(factor->size(), correspondences.size());

  // ds we have outliers - set robustifier
  factor->setRobustifier(&robustifier_saturated);

  // ds hook up factor, graph and solver
  graph->addFactor(factor);
  graph->bindFactors();
  ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));

  // ds set initial guess and optimize!
  X->setEstimate(Isometry3f::Identity());
  solver.compute();

  // ds validate optimization
  const auto& stats = solver.iterationStats();
  std::cerr << stats;
  ASSERT_EQ(stats.size(), static_cast<size_t>(maximum_number_of_iterations));
  ASSERT_LT(stats.back().chi_inliers, 250);

  // ds evaluate estimation error
  const Vector6f relative_error = geometry3d::t2tnq(X->estimate().inverse() * moving_in_fixed);
  ASSERT_LT(relative_error.norm(), 0.06);
}

TEST_F(Pinhole, SE3ProjectiveDepthErrorFactorCorrespondenceDriven_Noise_Outliers) {
  generateProblemPnP(1000 /*number of measurements*/, 1 /*noise magnitude*/, 100 /*# outliers*/);

  // ds create a stereo posit factor
  std::shared_ptr<SE3ProjectiveDepthErrorFactorCorrespondenceDriven> factor(
    new SE3ProjectiveDepthErrorFactorCorrespondenceDriven());
  factor->setVariableId(0, 0);

  // setup the factor
  factor->setEnabled(true);
  factor->setCameraMatrix(camera_calibration_matrix);
  factor->setImageDim(image_dimension);
  factor->setFixed(points_fixed_mono_depth);
  factor->setMoving(points_moving);
  factor->setCorrespondences(correspondences);
  Matrix3f information_matrix(Matrix3f::Identity());
  information_matrix(2, 2) = 10; // ds weight depth error higher (meters vs pixels)
  factor->setInformationMatrix(information_matrix);
  ASSERT_EQ(factor->size(), correspondences.size());

  // ds we have outliers - set robustifier
  factor->setRobustifier(&robustifier_saturated);

  // ds hook up factor, graph and solver
  graph->addFactor(factor);
  graph->bindFactors();
  ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));

  // ds set initial guess and optimize!
  X->setEstimate(Isometry3f::Identity());
  solver.compute();

  // ds validate optimization
  const auto& stats = solver.iterationStats();
  std::cerr << stats;
  ASSERT_EQ(stats.size(), static_cast<size_t>(maximum_number_of_iterations));
  ASSERT_LT(stats.back().chi_inliers, 250);

  // ds evaluate estimation error
  const Vector6f relative_error = geometry3d::t2tnq(X->estimate().inverse() * moving_in_fixed);
  ASSERT_LT(relative_error.norm(), 0.06);
}

TEST_F(Pinhole, SE3RectifiedStereoProjectiveErrorFactorCorrespondenceDriven_Noise_Outliers) {
  generateProblemPnP(1000 /*number of measurements*/, 1 /*noise magnitude*/, 100 /*# outliers*/);

  // ds create a stereo posit factor
  std::shared_ptr<SE3RectifiedStereoProjectiveErrorFactorCorrespondenceDriven> factor(
    new SE3RectifiedStereoProjectiveErrorFactorCorrespondenceDriven());
  factor->setVariableId(0, 0);

  // setup the factor
  factor->setEnabled(true);
  factor->setCameraMatrix(camera_calibration_matrix);
  factor->setImageDim(image_dimension);
  factor->setBaselinePixels(baseline_pixels);
  factor->setFixed(points_fixed_stereo);
  factor->setMoving(points_moving);
  factor->setCorrespondences(correspondences);
  Matrix3f information_matrix(Matrix3f::Identity());
  information_matrix(1, 1) = 2; // ds weight error in v twice (rectified)
  factor->setInformationMatrix(information_matrix);
  ASSERT_EQ(factor->size(), correspondences.size());

  // ds we have outliers - set robustifier
  factor->setRobustifier(&robustifier_saturated);

  // ds hook up factor, graph and solver
  graph->addFactor(factor);
  graph->bindFactors();
  ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));

  // ds set initial guess and optimize!
  X->setEstimate(Isometry3f::Identity());
  solver.compute();

  // ds validate optimization
  const auto& stats = solver.iterationStats();
  std::cerr << stats;
  ASSERT_EQ(stats.size(), static_cast<size_t>(maximum_number_of_iterations));
  ASSERT_LT(stats.back().chi_inliers, 500);

  // ds evaluate estimation error
  const Vector6f relative_error = geometry3d::t2tnq(X->estimate().inverse() * moving_in_fixed);
  ASSERT_LT(relative_error.norm(), 0.06);
}
