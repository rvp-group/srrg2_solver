#include "srrg_solver/variables_and_factors/types_2d/instances.h"
#include "srrg_solver/variables_and_factors/types_3d/instances.h"
#include "srrg_solver/variables_and_factors/types_calib/instances.h"
#include "srrg_solver/utils/factor_graph_utils/factor_graph_visit.h"
#include "srrg_solver/solver_core/internals/linear_solvers/instances.h"
#include "srrg_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholesky_csparse.h"
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/iteration_algorithm_lm.h"
#include "srrg_solver/solver_core/solver.h"
#include <fstream>
#include <srrg_config/configurable_manager.h>
#include <srrg_system_utils/parse_command_line.h>

#include "srrg_solver/utils/factor_graph_utils/instances.h"

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace std;

void initTypes() {
  registerTypes2D();
  registerTypes3D();
  calib_registerTypes();
  solver_registerTypes();
  linear_solver_registerTypes();
  solver_utils_registerTypes();
}

const char* banner[] = {
  "example of factor graph visit",
};

const std::string example_folder(SRRG2_SOLVER_EXAMPLE_FOLDER);
const std::string exe_name = "example_factor_graph_visit";
#define LOG std::cerr << exe_name + "|"

void generateConfig(const std::string& config_file, const std::string& config_name) {
  ConfigurableManager manager;
  auto policy       = manager.create<FactorGraphVisitPolicyBase>();
  auto cost_uniform = manager.create<FactorGraphVisitCostUniform>();
  auto visit        = manager.create<FactorGraphVisit>();

  policy->param_cost_function.setValue(cost_uniform);
  visit->param_cost_policies.pushBack(policy);
  visit->setName(config_name);

  // we now save the configuration
  manager.write(config_file);

  std::cerr << "configuration written on file " << config_file << std::endl;
}

int main(int argc, char** argv) {
  initTypes();

  ParseCommandLine cmd_line(argv);
  ArgumentFlag gen_config(&cmd_line, "j", "genrate-config", "generates a config file and quits");
  ArgumentString config_file(
    &cmd_line, "c", "config-file", "config file to read/write", "factor_graph_visit.json");
  ArgumentString config_name(
    &cmd_line, "n", "config-name", "name of the config to load", "my_visit");
  cmd_line.parse();

  if (gen_config.isSet()) {
    generateConfig(config_file.value(), config_name.value());
    return 0;
  }

  ConfigurableManager manager;
  manager.read(config_file.value());

  std::shared_ptr<FactorGraphVisit> visit =
    manager.getByName<FactorGraphVisit>(config_name.value());
  if (!visit) {
    cerr << "cast fail" << endl;
    return -1;
  }
  LOG << "factor graph visit correctly loaded" << endl;

  // ia loading file, if no input specified, load default one
  std::string input_file = "";
  cmd_line.parse();
  if (cmd_line.lastParsedArgs().empty()) {
    LOG << "no input file, loading default one" << std::endl;
    input_file = example_folder + "/data/pose_graph_100.boss";
  } else {
    input_file = cmd_line.lastParsedArgs()[0];
  }

  LOG << "loading file [" << FG_YELLOW(input_file) << "]" << std::endl;

  // loading graph
  FactorGraphPtr graph = FactorGraph::read(input_file);
  if (!graph) {
    cerr << "no graph found in file, [" << input_file << "], aborting" << endl;
    return -1;
  }

  LOG << "loaded [" << input_file << "], "
      << " vars:" << graph->variables().size() << " fact:" << graph->factors().size() << endl;

  // after loading one might want to bind the values
  // this "links" the variables to the factors
  // based on the respective ids
  // this is automatically done by the solver...
  // but this is an example
  // binding adds new variables if some factor references
  // a non instantiated variable
  // this should not be the case
  // shouldn't be the case with a standard g2o file
  LOG << "binding... ";
  int new_vars = graph->bindFactors();
  cerr << "created " << new_vars << " variables" << endl;

  // tell the solver which graph to optimize
  visit->setGraph(*graph);

  if (!graph->variables().size()) {
    LOG << "no variables, no visit" << endl;
    return -1;
  }
  graph->bindFactors();

  std::vector<VariableBase::Id> sources;
  sources.push_back(graph->variables().begin().key());

  LOG << "starting visit from node: " << sources[0] << std::endl;

  visit->setSources(sources);

  visit->compute();

  LOG << "visit completed" << std::endl;

  return 0;
}
