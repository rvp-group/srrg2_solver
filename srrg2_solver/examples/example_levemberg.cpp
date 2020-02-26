#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/iteration_algorithm_lm.h"
#include "srrg_solver/solver_core/solver.h"

#include "srrg_solver/variables_and_factors/types_2d/instances.h"
using namespace srrg2_core;
using namespace srrg2_solver;

int main(int argc, char** argv) {
  using FactorType           = SE2PosePointBearingErrorFactor;
  using VariablePoseType     = VariableSE2Right;
  using VariablePointType    = VariablePoint2;
  using VariableSE2Vector    = std::vector<VariablePoseType*>;
  using VariablePoint2Vector = std::vector<VariablePointType*>;
  using FactorVector         = std::vector<FactorType*>;
  using Vector2fVector       = std::vector<Vector2f, Eigen::aligned_allocator<Vector2f>>;
  using Isometry2fVector     = std::vector<Isometry2f, Eigen::aligned_allocator<Isometry2f>>;

  size_t num_landmarks = 200;
  size_t num_poses     = 100;

  // tg containers
  VariableSE2Vector poses;
  poses.reserve(num_poses);
  VariablePoint2Vector landmarks;
  landmarks.reserve(num_landmarks);

  Isometry2fVector ground_truth_poses;
  ground_truth_poses.reserve(num_poses);
  Vector2fVector ground_truth_landmarks;
  ground_truth_landmarks.reserve(num_landmarks);

  Vector3f pose_perturbation     = Vector3f::Zero();
  Vector2f landmark_perturbation = Vector2f::Zero();
  Vector3f pose_vector           = Vector3f::Zero();
  Vector2f landmark              = Vector2f::Zero();

  FactorGraphPtr graph(new FactorGraph);

  // tg create an anchor for the graph
  VariablePoseType* v = new VariablePoseType;
  pose_vector.setRandom();
  Isometry2f T = geometry2d::v2t(pose_vector);
  ground_truth_poses.emplace_back(T);
  v->setEstimate(T);
  v->setStatus(VariableBase::Status::Fixed);
  poses.emplace_back(v);
  graph->addVariable(VariableBasePtr(v));

  // tg generate random poses
  for (size_t pose_idx = 1; pose_idx < num_poses; ++pose_idx) {
    pose_vector.setRandom();
    T = geometry2d::v2t(pose_vector);
    ground_truth_poses.emplace_back(T);
    pose_perturbation.setRandom();
    v = new VariablePoseType;
    v->setEstimate(T * geometry2d::v2t(pose_perturbation));
    poses.emplace_back(v);
    graph->addVariable(VariableBasePtr(v));
  }

  std::cerr << "Poses done" << std::endl;

  // tg generate random landmarks
  VariablePointType* point = nullptr;
  for (size_t landmark_idx = 0; landmark_idx < num_landmarks; ++landmark_idx) {
    landmark.setRandom();
    ground_truth_landmarks.emplace_back(landmark);
    landmark_perturbation.setRandom();
    point = new VariablePointType;
    landmarks.emplace_back(point);
    point->setEstimate(landmark + landmark_perturbation);
    graph->addVariable(VariableBasePtr(point));
  }
  std::cerr << "Landmarks done " << std::endl;
  // tg all landmark are visible from every pose
  FactorType* f = nullptr;
  FactorVector factors;
  factors.reserve(num_poses * num_landmarks);
  for (size_t p = 0; p < num_poses; ++p) {
    for (size_t l = 0; l < num_landmarks; ++l) {
      f = new FactorType;
      f->setVariableId(0, poses[p]->graphId());
      f->setVariableId(1, landmarks[l]->graphId());
      const Isometry2f& T            = ground_truth_poses[p];
      const Vector2f& point          = ground_truth_landmarks[l];
      const Vector2f predicted_point = T.inverse() * point;
      float angle                    = std::atan2(predicted_point(1), predicted_point(0));
      f->setMeasurement(Vector1f(angle));
      graph->addFactor(FactorBasePtr(f));
      factors.emplace_back(f);
    }
  }
  std::cerr << "Factors done " << std::endl;
  // tg optimize graph
  Solver solver;
  size_t num_iterations = 40;
  solver.param_max_iterations.pushBack(num_iterations);
  solver.param_termination_criteria.setValue(0);
  std::shared_ptr<IterationAlgorithmLM> lm(new IterationAlgorithmLM);
  solver.param_algorithm.setValue(lm);
  solver.setGraph(graph);
  solver.compute();

  const auto& stats = solver.iterationStats();

  std::cerr << stats << std::endl;
}