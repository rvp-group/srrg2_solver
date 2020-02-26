#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>

// ia include linear solver stuff
#include "srrg_solver/solver_core/internals/linear_solvers/instances.h"
// ia include solver stuff (instances)
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/solver.h"
// ia include types stuff (instances)
#include "srrg_solver/variables_and_factors/types_3d/instances.h"

#include <random>

const std::string exe_name = "example_se3_pgo";
#define LOG std::cerr << exe_name + "|"

using namespace srrg2_core;
using namespace srrg2_solver;

// ia register all the types (REQUIRED)
void registerAllTypes() {
  registerTypes3D();
  solver_registerTypes();
  linear_solver_registerTypes();
}

// ia in this example we will have PosePose factor. Here we choose to use
// ia a factor with AD. The Solver uses factors with more than one
// ia variable)
using FactorType         = SE3PosePoseGeodesicErrorFactor;
using VariableType       = VariableSE3QuaternionRight;
using Isometry3fVector = std::vector<Isometry3f, Eigen::aligned_allocator<Isometry3f>>;

// ia aux function that creates a manhattan-like trajectory
void createGTTrajectory(const size_t& n_poses_, Isometry3fVector& gt_trajectory_);
void createFactorGraph(const Isometry3fVector& gt_trajectory_,
                         const FactorGraphPtr& graph_);

int main(int argc, char** argv) {
  registerAllTypes();

  ParseCommandLine cmd_line(argv);
  ArgumentInt num_poses(
    &cmd_line, "n", "num-poses", "number of poses to optimize (aka vertices)", 100);
  ArgumentInt solver_iterations(&cmd_line, "i", "iterations", "number of solver iterations", 20);
  ArgumentFlag no_guess(&cmd_line, "ng", "no-guess", "start from the optimum");
  cmd_line.parse();

  // ia create a GT trajectory
  Isometry3fVector gt_trajectory;
  createGTTrajectory(num_poses.value(), gt_trajectory);

  // ia create a graph
  FactorGraphPtr graph(new FactorGraph);
  createFactorGraph(gt_trajectory, graph);

  // ia bind factors and variables
  graph->bindFactors();

  size_t n_variables = graph->variables().size();
  size_t n_factors   = graph->factors().size();
  LOG << "created [" << FG_YELLOW(n_variables) << "] variables "
      << "and [" << FG_YELLOW(n_factors) << "] factors" << std::endl;

  // ia create a wrong initial guess - in this case we set all the variables to the origin
  if (!no_guess.isSet()) {
    for (size_t i = 1; i < n_variables; ++i) {
      static_cast<VariableType*>(graph->variable(i))
        ->setEstimate(Isometry3f::Identity());
    }
  }

  // ia create a solver and do its setup
  Solver solver;
  solver.param_termination_criteria.setValue(nullptr);
  solver.param_max_iterations.pushBack(solver_iterations.value());
  solver.setGraph(graph);

  // ia do the optimization
  solver.compute();
  // ia log
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

  //  std::cerr << "T[" << 0 << "]\n" << previous_pose.matrix() << std::endl;
  for (size_t i = 1; i < n_poses_; ++i) {
    const Vector6f current_pose = geometry3d::t2ta(previous_pose);
    Vector6f next_pose          = Vector6f::Zero();

    // ia sample new motion direction
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

    previous_pose = next_T;

    //    std::cerr << "T[" << i << "]\n" << next_T.matrix() << std::endl;
  }

  //  std::cerr << "gt trajectory size = " << gt_trajectory_.size() << std::endl;
}

void createFactorGraph(const Isometry3fVector& gt_trajectory_, const FactorGraphPtr& graph_) {
  using OmegaType = typename FactorType::InformationMatrixType;

  VariableType* current_variable = new VariableType();
  current_variable->setGraphId(0);
  current_variable->setEstimate(gt_trajectory_[0]);
  current_variable->setStatus(VariableBase::Status::Fixed);
  graph_->addVariable(VariableBasePtr(current_variable));

  for (size_t i = 1; i < gt_trajectory_.size(); ++i) {
    const size_t curr_id     = i;
    const auto& current_pose = gt_trajectory_[curr_id];

    current_variable = new VariableType();
    current_variable->setGraphId(i);
    current_variable->setEstimate(current_pose);
    graph_->addVariable(VariableBasePtr(current_variable));

    const size_t prev_id            = i - 1;
    VariableType* prev_variable = static_cast<VariableType*>(graph_->variable(prev_id));
    const Isometry3f Z_odom_gt = prev_variable->estimate().inverse() * current_variable->estimate();
    const OmegaType odom_information = OmegaType::Identity();

    FactorType* odom_factor = new FactorType();
    odom_factor->setVariableId(0, prev_variable->graphId());
    odom_factor->setVariableId(1, current_variable->graphId());
    odom_factor->setMeasurement(Z_odom_gt);
    odom_factor->setInformationMatrix(odom_information);
    graph_->addFactor(FactorBasePtr(odom_factor));

    for (const auto& id_var_pair : graph_->variables()) {
      if ((size_t)(id_var_pair.first) == curr_id) {
        continue;
      }

      const VariableType* var = static_cast<const VariableType*>(id_var_pair.second);
      const Isometry3f delta      = var->estimate().inverse() * current_variable->estimate();
      const Vector6f delta_vector = geometry3d::t2tnq(delta);
      const Vector3f delta_trans  = delta_vector.head(3);
      if (delta_trans.norm() < 2.5f) {
        const OmegaType closure_information = OmegaType::Identity();
        FactorType* closure_factor      = new FactorType();
        closure_factor->setVariableId(0, var->graphId());
        closure_factor->setVariableId(1, current_variable->graphId());
        closure_factor->setMeasurement(delta);
        closure_factor->setInformationMatrix(closure_information);
        graph_->addFactor(FactorBasePtr(closure_factor));
      }
    }
  }
}
