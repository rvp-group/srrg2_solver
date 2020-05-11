#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/internals/linear_solvers/instances.h"
#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/variables_and_factors/types_3d/instances.h"
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>

#include <random>
// Executable name and log
const std::string exe_name = "example_se3_pgo";
#define LOG std::cerr << exe_name + "|"

using namespace srrg2_core;
using namespace srrg2_solver;

// Register all the types
void registerAllTypes() {
  variables_and_factors_3d_registerTypes();
  solver_registerTypes();
  linear_solver_registerTypes();
}

// Define types to simplify the code
using FactorType       = SE3PosePoseGeodesicErrorFactor;
using VariableType     = VariableSE3QuaternionRight;
using Isometry3fVector = std::vector<Isometry3f, Eigen::aligned_allocator<Isometry3f>>;

// Auxiliary function that creates a manhattan-like trajectory
void createGTTrajectory(const size_t& n_poses_, Isometry3fVector& gt_trajectory_);
// Create a factor graph out of a trjectory
void createFactorGraph(const Isometry3fVector& gt_trajectory_, const FactorGraphPtr& graph_);

int main(int argc, char** argv) {
  registerAllTypes();
  // Parser for the command line to interact with the program - see srrg2_core
  ParseCommandLine cmd_line(argv);
  ArgumentInt num_poses(
    &cmd_line, "n", "num-poses", "number of poses to optimize (aka vertices)", 100);
  ArgumentInt solver_iterations(&cmd_line, "i", "iterations", "number of solver iterations", 20);
  ArgumentFlag no_guess(&cmd_line, "ng", "no-guess", "start from the optimum");
  cmd_line.parse();

  // Create the GT trajectory
  Isometry3fVector gt_trajectory;
  createGTTrajectory(num_poses.value(), gt_trajectory);

  // Instanciate and feel the factor graph
  FactorGraphPtr graph(new FactorGraph);
  createFactorGraph(gt_trajectory, graph);

  // Connect factors to variables
  graph->bindFactors();

  size_t n_variables = graph->variables().size();
  size_t n_factors   = graph->factors().size();
  LOG << "created [" << FG_YELLOW(n_variables) << "] variables "
      << "and [" << FG_YELLOW(n_factors) << "] factors" << std::endl;

  // if no_guess is not set we put all the variables to the origin
  if (!no_guess.isSet()) {
    for (size_t i = 1; i < n_variables; ++i) {
      static_cast<VariableType*>(graph->variable(i))->setEstimate(Isometry3f::Identity());
    }
  }

  // Instanciate a solver
  Solver solver;
  // Configure the solver - Set max iterations and remove termination criteria (Default is
  // SimpleTerminationCriteria)
  solver.param_termination_criteria.setValue(nullptr);
  solver.param_max_iterations.pushBack(solver_iterations.value());
  // Connect the graph to the solver and compute
  solver.setGraph(graph);
  solver.compute();
  // Visualize statistics and exit
  const auto& stats = solver.iterationStats();
  LOG << "performed [" << FG_YELLOW(stats.size()) << "] iterations" << std::endl;
  LOG << "stats\n\n";
  std::cerr << stats << std::endl;
  return 0;
}

void createGTTrajectory(const size_t& n_poses_, Isometry3fVector& gt_trajectory_) {
  gt_trajectory_.clear();
  gt_trajectory_.reserve(n_poses_);

  std::mt19937 rnd_generator;
  std::uniform_real_distribution<float> uniform_distribution(0.0, 1.0);

  Isometry3f previous_pose = Isometry3f::Identity();
  gt_trajectory_.emplace_back(previous_pose);

  for (size_t i = 1; i < n_poses_; ++i) {
    const Vector6f current_pose = geometry3d::t2ta(previous_pose);
    Vector6f next_pose          = Vector6f::Zero();

    // Sample new motion direction
    const float dir_selector  = uniform_distribution(rnd_generator);
    const float current_theta = current_pose[5];
    float next_theta = 0, next_x = 0, next_y = 0;
    // Move to the next pose
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
    // ta2t is a function that maps a 6D vector (first 3 components represents the translation, the
    // other three are euler angles) to an element of SE3 - See srrg2_core
    const Isometry3f next_T = geometry3d::ta2t(next_pose);
    gt_trajectory_.emplace_back(next_T);

    previous_pose = next_T;
  }
}

void createFactorGraph(const Isometry3fVector& gt_trajectory_, const FactorGraphPtr& graph_) {
  using OmegaType = typename FactorType::InformationMatrixType;
  // Set and fix the first variable in the graph
  VariableType* current_variable = new VariableType();
  current_variable->setGraphId(0);
  current_variable->setEstimate(gt_trajectory_[0]);
  current_variable->setStatus(VariableBase::Status::Fixed);
  graph_->addVariable(VariableBasePtr(current_variable));

  for (size_t i = 1; i < gt_trajectory_.size(); ++i) {
    const size_t curr_id     = i;
    const auto& current_pose = gt_trajectory_[curr_id];
    // for each element in the trajectory add a variable in the graph
    current_variable = new VariableType();
    current_variable->setGraphId(i);
    current_variable->setEstimate(current_pose);
    graph_->addVariable(VariableBasePtr(current_variable));
    // Create measurements from 2 successive poses
    const size_t prev_id        = i - 1;
    VariableType* prev_variable = static_cast<VariableType*>(graph_->variable(prev_id));
    const Isometry3f Z_odom_gt = prev_variable->estimate().inverse() * current_variable->estimate();
    const OmegaType odom_information = OmegaType::Identity();
    // Add PosePose factors
    FactorType* odom_factor = new FactorType();
    // Set variable indices in the internal container of the factor with
    // the corresponding graph id of the variable
    odom_factor->setVariableId(0, prev_variable->graphId());
    odom_factor->setVariableId(1, current_variable->graphId());
    // Add measurement to the factor
    odom_factor->setMeasurement(Z_odom_gt);
    odom_factor->setInformationMatrix(odom_information);
    // Add factor to the graph
    graph_->addFactor(FactorBasePtr(odom_factor));
    // Check for closures
    for (const auto& id_var_pair : graph_->variables()) {
      if ((size_t)(id_var_pair.first) == curr_id) {
        continue;
      }

      const VariableType* var = static_cast<const VariableType*>(id_var_pair.second);
      const Isometry3f delta  = var->estimate().inverse() * current_variable->estimate();
      // Add a closure if the relative position of two robot poses is
      // below 2.5 meters
      const Vector6f delta_vector = geometry3d::t2tnq(delta);
      const Vector3f delta_trans  = delta_vector.head(3);
      if (delta_trans.norm() < 2.5f) {
        const OmegaType closure_information = OmegaType::Identity();
        FactorType* closure_factor          = new FactorType();
        // Setup factor indices and graph ids for the variables
        closure_factor->setVariableId(0, var->graphId());
        closure_factor->setVariableId(1, current_variable->graphId());
        closure_factor->setMeasurement(delta);
        closure_factor->setInformationMatrix(closure_information);
        // Add closure to graph
        graph_->addFactor(FactorBasePtr(closure_factor));
      }
    }
  }
}
