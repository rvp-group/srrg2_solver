#include "instances.h"
#include "calib_graph_2d_assembler.h"
#include "calib_graph_3d_assembler.h"

namespace srrg2_solver {
  void solver_calib_utils_registerTypes() {
    BOSS_REGISTER_CLASS(CalibGraph2DAssembler);
    BOSS_REGISTER_CLASS(CalibGraph3DAssembler);
  }
} // namespace srrg2_solver
