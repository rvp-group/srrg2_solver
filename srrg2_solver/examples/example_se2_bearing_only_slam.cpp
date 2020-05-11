#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/iteration_algorithm_lm.h"
#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/variables_and_factors/types_2d/instances.h"

using namespace srrg2_core;
using namespace srrg2_solver;

// Executable name and log
const std::string exe_name = "example_se2_bearing_only_slam";
#define LOG std::cerr << exe_name + "|"

// Define types to simplify the code
using FactorType        = SE2PosePointBearingErrorFactor;
using VariablePoseType  = VariableSE2Right;
using VariablePointType = VariablePoint2;
using Vector2fVector    = std::vector<Vector2f, Eigen::aligned_allocator<Vector2f>>;
using Isometry2fVector  = std::vector<Isometry2f, Eigen::aligned_allocator<Isometry2f>>;

// Number of poses and landmarks
size_t num_landmarks = 200;
size_t num_poses     = 100;

// Generate synthetic world and fill the factor graph
void generateProblem(FactorGraphPtr& graph);

int main(int argc, char** argv) {
  // Instanciate a factor graph
  FactorGraphPtr graph(new FactorGraph);
  // Fill the graph
  generateProblem(graph);
  // Instanciate a solver
  Solver solver;
  // Configure the solver - Set max iterations, termination criteria and algorithm
  size_t num_iterations = 25;

  solver.param_max_iterations.pushBack(num_iterations);
  solver.param_termination_criteria.setValue(
    TerminationCriteriaPtr(new PerturbationNormTerminationCriteria));
  solver.param_algorithm.setValue(IterationAlgorithmBasePtr(new IterationAlgorithmLM));
  // Connect graph to the solver and compute
  solver.setGraph(graph);
  solver.compute();
  // Visualize statistics and exit
  const auto& stats = solver.iterationStats();
  std::cerr << stats << std::endl;
  return 0;
}

void generateProblem(FactorGraphPtr& graph) {
  // Define containers for poses, landmarks
  Isometry2fVector ground_truth_poses;
  ground_truth_poses.reserve(num_poses);
  Vector2fVector ground_truth_landmarks;
  ground_truth_landmarks.reserve(num_landmarks);
  // Initialize quantities used in the loops
  Vector3f pose_perturbation     = Vector3f::Zero();
  Vector2f landmark_perturbation = Vector2f::Zero();
  Vector3f pose_vector           = Vector3f::Zero();
  Vector2f landmark              = Vector2f::Zero();

  // Create an anchor for the graph
  VariablePoseType* v = new VariablePoseType;
  pose_vector.setRandom();
  // v2t is a function that maps a 3D vector to an element of SE2 - See srrg2_core
  Isometry2f T = geometry2d::v2t(pose_vector);
  ground_truth_poses.emplace_back(T);
  v->setEstimate(T);
  v->setStatus(VariableBase::Status::Fixed);
  graph->addVariable(VariableBasePtr(v));

  // Generate random poses and associated variables
  for (size_t pose_idx = 1; pose_idx < num_poses; ++pose_idx) {
    pose_vector.setRandom();
    T = geometry2d::v2t(pose_vector);
    ground_truth_poses.emplace_back(T);
    // The initial guess for the poses is a perturbed verstion of the GT
    v = new VariablePoseType;
    pose_perturbation.setRandom();
    v->setEstimate(T * geometry2d::v2t(pose_perturbation));
    // Add variable to the graph
    graph->addVariable(VariableBasePtr(v));
  }
  // Generate random landmarks
  VariablePointType* point = nullptr;
  for (size_t landmark_idx = 0; landmark_idx < num_landmarks; ++landmark_idx) {
    landmark.setRandom();
    ground_truth_landmarks.emplace_back(landmark);
    landmark_perturbation.setRandom();
    // The initial guess for the landmark is a perturbed version of the GT
    point = new VariablePointType;
    point->setEstimate(landmark + landmark_perturbation);
    // Add variable to the graph
    graph->addVariable(VariableBasePtr(point));
  }

  // All landmark are visible from every pose
  FactorType* f = nullptr;
  for (size_t p = 0; p < num_poses; ++p) {
    for (size_t l = 0; l < num_landmarks; ++l) {
      f = new FactorType;
      // Set the indices of the variables in the internal container of the factor
      // and graph ids
      f->setVariableId(0, p);
      f->setVariableId(1, l + num_poses);
      // Generate measurement
      const Isometry2f& T            = ground_truth_poses[p];
      const Vector2f& point          = ground_truth_landmarks[l];
      const Vector2f predicted_point = T.inverse() * point;
      float angle                    = std::atan2(predicted_point(1), predicted_point(0));
      // Add factor to the graph
      f->setMeasurement(Vector1f(angle));
      graph->addFactor(FactorBasePtr(f));
    }
  }
}
