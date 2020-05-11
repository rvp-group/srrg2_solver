#include "srrg_solver/variables_and_factors/types_2d/se2_pose_pose_geodesic_error_factor.h"
#include "srrg_solver/variables_and_factors/types_2d/instances.h"
#include "srrg_solver/solver_core/internals/linear_solvers/instances.h"
#include "srrg_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholesky_csparse.h"
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/iteration_algorithm_lm.h"
#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/utils/factor_graph_utils/factor_graph_closure_validator.h"

using namespace srrg2_solver;
using namespace srrg2_core;

using FactorSE2= SE2PosePoseGeodesicErrorFactor;

void initTypes() {
  variables_and_factors_2d_registerTypes();
  solver_registerTypes();
  linear_solver_registerTypes();
}

void printClosures(const FactorGraphClosureValidator::ClosureStatsMap& closures) {
  for (auto c_it = closures.begin(); c_it != closures.end(); ++c_it) {
    auto& stats = c_it->second;
    std::cerr << "f: " << c_it->first << " n_checks: " << stats.num_times_checked
              << " n_ok: " << stats.num_times_good << std::endl;
  }
}

void addCircle(FactorGraphPtr graph,
               float radius,
               int segments,
               int start_id             = 0,
               const Isometry2f& origin = Isometry2f::Identity()) {
  std::vector<std::shared_ptr<VariableSE2RightAD>> variables(segments);
  float alpha    = 0;
  float d_alpha  = 2 * M_PI / segments;
  int current_id = start_id;
  for (int i = 0; i < segments; ++i, ++current_id) {
    VariableSE2RightAD* v = new VariableSE2RightAD;
    Isometry2f pose;
    float s            = sin(alpha);
    float c            = cos(alpha);
    pose.translation() = Vector2f(c, s) * radius;
    pose.linear() << c, s, -s, c;
    v->setEstimate(origin * pose);
    v->setGraphId(current_id);
    variables[i] = std::shared_ptr<VariableSE2RightAD>(v);
    alpha += d_alpha;
    graph->addVariable(variables[i]);
    if (i > 0) {
      FactorSE2* f = new FactorSE2;
      f->setMeasurement(variables[i - 1]->estimate().inverse() * v->estimate());
      f->setVariableId(0, current_id - 1);
      f->setVariableId(1, current_id);
      graph->addFactor(FactorBasePtr(f));
    }
    if (i == segments - 1) {
      FactorSE2* f = new FactorSE2;
      f->setMeasurement(variables[0]->estimate().inverse() * v->estimate());
      f->setVariableId(0, start_id);
      f->setVariableId(1, current_id);
      graph->addFactor(FactorBasePtr(f));
    }
  }
}

int main(int argc, char** argv) {
  initTypes();
  std::vector<std::shared_ptr<FactorBase>> closures;
  FactorGraphPtr graph(new FactorGraph);
  addCircle(graph, 5, 10);
  Isometry2f iso;
  iso.setIdentity();
  iso.linear() = Eigen::Rotation2Df(M_PI).toRotationMatrix();
  iso.translation() << 10, 10;
  addCircle(graph, 5, 10, graph->variables().end().key(), iso);
  iso.translation() << 20, 20;
  addCircle(graph, 5, 10, graph->variables().end().key(), iso);

  FactorSE2* outlier = new FactorSE2;
  outlier->setMeasurement(Isometry2f::Identity());
  outlier->setVariableId(0, 0);
  outlier->setVariableId(1, 15);
  FactorBasePtr outlier_(outlier);
  graph->addFactor(outlier_);
  closures.push_back(outlier_);

  // 0_to_1
  iso.setIdentity();
  {
    int start                        = 0;
    int offset                       = 10;
    FactorSE2* f = new FactorSE2;
    f->setMeasurement(iso);
    f->setVariableId(0, start);
    f->setVariableId(1, start + offset);
    FactorBasePtr _f(f);
    graph->addFactor(_f);
    closures.push_back(_f);
  }
  {
    int start                        = 2;
    int offset                       = 10;
    FactorSE2* f = new FactorSE2;
    f->setMeasurement(iso);
    f->setVariableId(0, start);
    f->setVariableId(1, start + offset);
    FactorBasePtr _f(f);
    graph->addFactor(_f);
    closures.push_back(_f);
  }

  {
    int start                        = 3;
    int offset                       = 10;
    FactorSE2* f = new FactorSE2;
    f->setMeasurement(iso);
    f->setVariableId(0, start);
    f->setVariableId(1, start + offset);
    FactorBasePtr _f(f);
    graph->addFactor(_f);
    closures.push_back(_f);
  }

  // 0 to 2
  {
    int start                        = 0;
    int offset                       = 20;
    FactorSE2* f = new FactorSE2;
    f->setMeasurement(iso);
    f->setVariableId(0, start);
    f->setVariableId(1, start + offset);
    FactorBasePtr _f(f);
    graph->addFactor(_f);
    closures.push_back(_f);
  }
  {
    int start                        = 2;
    int offset                       = 20;
    FactorSE2* f = new FactorSE2;
    f->setMeasurement(iso);
    f->setVariableId(0, start);
    f->setVariableId(1, start + offset);
    FactorBasePtr _f(f);
    graph->addFactor(_f);
    closures.push_back(_f);
  }

  {
    int start                        = 3;
    int offset                       = 20;
    FactorSE2* f = new FactorSE2;
    f->setMeasurement(iso);
    f->setVariableId(0, start);
    f->setVariableId(1, start + offset);
    FactorBasePtr _f(f);
    graph->addFactor(_f);
    closures.push_back(_f);
  }

  // 1 to 2
  {
    int start                        = 10;
    int offset                       = 10;
    FactorSE2* f = new FactorSE2;
    f->setMeasurement(iso);
    f->setVariableId(0, start);
    f->setVariableId(1, start + offset);
    FactorBasePtr _f(f);
    graph->addFactor(_f);
    closures.push_back(_f);
  }
  {
    int start                        = 12;
    int offset                       = 10;
    FactorSE2* f = new FactorSE2;
    f->setMeasurement(iso);
    f->setVariableId(0, start);
    f->setVariableId(1, start + offset);
    FactorBasePtr _f(f);
    graph->addFactor(_f);
    closures.push_back(_f);
  }

  {
    int start                        = 13;
    int offset                       = 10;
    FactorSE2* f = new FactorSE2;
    f->setMeasurement(iso);
    f->setVariableId(0, start);
    f->setVariableId(1, start + offset);
    FactorBasePtr _f(f);
    graph->addFactor(_f);
    closures.push_back(_f);
  }

  graph->bindFactors();

  graph->write("lc.json");
  FactorGraphClosureValidator validator;
  validator.param_partition_expansion_range.setValue(100);
  validator.setGraph(graph);
  for (size_t i = 0; i < closures.size(); ++i)
    validator.addClosure(std::shared_ptr<FactorBase>(closures[i]));

  std::cerr << "DoValidate, round1" << std::endl;
  validator.compute(graph->variable(0));
  std::cerr << "partitions: " << validator.partitionsNum() << std::endl;
  printClosures(validator.closures());

  std::cerr << "DoValidate, round2" << std::endl;
  validator.compute(graph->variable(20));
  std::cerr << "partitions: " << validator.partitionsNum() << std::endl;
  printClosures(validator.closures());

  std::cerr << "DoValidate, round3" << std::endl;
  validator.compute(graph->variable(2));
  std::cerr << "partitions: " << validator.partitionsNum() << std::endl;
  printClosures(validator.closures());

  std::cerr << "DoValidate, round4" << std::endl;
  validator.compute(graph->variable(10));
  std::cerr << "partitions: " << validator.partitionsNum() << std::endl;
  printClosures(validator.closures());
}
