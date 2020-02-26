#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>

#include "srrg_solver/utils/g2o_converter/g2o_converter.h"

const std::string exe_name = "solver_app_graph_converter";
#define LOG std::cerr << exe_name + "|"

using namespace srrg2_core;
using namespace srrg2_solver;

int main(int argc, char** argv) {
  ParseCommandLine cmd_line(argv);
  ArgumentString argument_input_file(
    &cmd_line, "i", "input-file", "input graph to read and convert", "");
  ArgumentString argument_output_file(&cmd_line, "o", "output-file", "output graph converted", "");

  cmd_line.parse();

  if (!argument_input_file.isSet()) {
    throw std::runtime_error(exe_name + "|missing input file, exit");
  }

  if (!argument_output_file.isSet()) {
    throw std::runtime_error(exe_name + "|missing output file, exit");
  }

  G2OConverter converter;
  converter.loadGraph(argument_input_file.value());
  converter.writeGraph(argument_output_file.value());
}
