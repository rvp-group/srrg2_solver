#include "srrg_geometry/geometry_defs.h"
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/utils/g2o_converter/g2o_converter.h"
#include "srrg_test/test_helper.hpp"

using namespace srrg2_core;
using namespace srrg2_solver;

class G2OConverterSuite : public ::testing::Test {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  void SetUp() override;
  void TearDown() override;
  void populateGraph(FactorGraph& graph_);
  VariableBasePtr getPoseVariable();
  FactorBasePtr getPoseFactor();
};

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_);
}

TEST_F(G2OConverterSuite, ConvertBOSStoG2O) {
  G2OConverter converter;

  // ds populate a small graph and save it to disk
  FactorGraph graph;
  populateGraph(graph);
  graph.write("test_graph.boss");

  // ds run conversion from disk
  converter.loadGraph("test_graph.boss");
  converter.writeGraph("test_graph.g2o");
}

TEST_F(G2OConverterSuite, ConvertG2OtoBOSS) {
  G2OConverter generator;

  // ds populate a small graph and save it to disk
  FactorGraph graph;
  populateGraph(graph);
  graph.write("test_graph.boss");

  // ds run conversion from disk
  generator.loadGraph("test_graph.boss");
  generator.writeGraph("test_graph.g2o");

  G2OConverter converter;
  converter.loadGraph("test_graph.g2o");
  converter.writeGraph("test_graph.boss");
}

void G2OConverterSuite::SetUp() {
  solver_registerTypes();
  variables_and_factors_2d_registerTypes();
  variables_and_factors_3d_registerTypes();
}

void G2OConverterSuite::TearDown() {
  ASSERT_EQ(std::remove("test_graph.boss"), 0);
  ASSERT_EQ(std::remove("test_graph.g2o"), 0);
}

void G2OConverterSuite::populateGraph(FactorGraph& graph_) {
  // ds add first pose (fixed)
  VariableBasePtr variable_0 = getPoseVariable();
  variable_0->setGraphId(0);
  variable_0->setStatus(VariableBase::Status::Fixed);
  graph_.addVariable(VariableBasePtr(variable_0));

  // ds add second pose
  VariableBasePtr variable_1 = getPoseVariable();
  variable_1->setGraphId(1);
  graph_.addVariable(VariableBasePtr(variable_1));

  // ds add measurement between first and second pose
  FactorBasePtr factor = getPoseFactor();
  if (factor) {
    factor->setVariableId(0, variable_0->graphId());
    factor->setVariableId(1, variable_1->graphId());
    graph_.addFactor(factor);
  }
}
// tg TODO make the converter AD free
VariableBasePtr G2OConverterSuite::getPoseVariable() {
  VariableBasePtr variable             = nullptr;
  VariableSE3EulerLeft* variable_typed = new VariableSE3EulerLeft();
  variable_typed->setEstimate(Isometry3f::Identity());
  variable = VariableBasePtr(variable_typed);
  return variable;
}

FactorBasePtr G2OConverterSuite::getPoseFactor() {
  FactorBasePtr factor = nullptr;
  auto factor_typed    = new SE3PosePoseChordalEulerLeftErrorFactor();
  factor_typed->setMeasurement(Isometry3f::Identity());
  factor = FactorBasePtr(factor_typed);
  return factor;
}
