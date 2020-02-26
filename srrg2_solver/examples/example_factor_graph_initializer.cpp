#include "srrg_solver/variables_and_factors/types_2d/instances.h"
#include "srrg_solver/variables_and_factors/types_3d/instances.h"
#include "srrg_solver/variables_and_factors/types_calib/instances.h"
#include "srrg_solver/utils/factor_graph_utils/factor_graph_initializer.h"
#include "srrg_solver/utils/factor_graph_utils/instances.h"
#include "srrg_solver/solver_core/internals/linear_solvers/instances.h"
#include "srrg_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholesky_csparse.h"
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/iteration_algorithm_lm.h"
#include "srrg_solver/solver_core/solver.h"
#include <srrg_config/configurable_manager.h>
#include <srrg_system_utils/parse_command_line.h>

#include <fstream>

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
  "example of factor graph intialization",
};

const std::string example_folder(SRRG2_SOLVER_EXAMPLE_FOLDER);
const std::string exe_name = "example_factor_graph_initializer";
#define LOG std::cerr << exe_name + "|"

int main(int argc, char** argv) {
  initTypes();

  std::string input_file = "";

  ParseCommandLine cmd_line(argv);
  ArgumentString output_file(&cmd_line,
                             "o",
                             "output",
                             "name of the initialized file to save",
                             exe_name + "_output_graph.boss");

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

  cerr << "loaded [" << input_file << "], "
       << " vars:" << graph->variables().size() << " fact:" << graph->factors().size() << endl;

  FactorGraphInitializer initializer;
  initializer.setGraph(*graph);
  initializer.compute();

  if (output_file.value() != "") {
    cerr << "writing output to: [" << output_file.value() << "]" << endl;
    graph->write(output_file.value());
  } else {
    cerr << "no output file chosen terminating quietly" << endl;
  }

  return 0;
}
