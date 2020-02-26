#include "solver_action_draw.h"

namespace srrg2_solver_gui {

  void SolverActionDraw::doAction() {
    if (_canvas == nullptr) {
      throw std::runtime_error("SolverActionDraw::doAction|invalid canvas");
    }

    if (_graph == nullptr) {
      throw std::runtime_error("SolverActionDraw::doAction|invalid graph");
    }

    // ia write variables on the canvas
    for (const auto& v_tuple : _graph->variables()) {
      v_tuple.second->draw(_canvas);
    }

    for (const auto& f : _graph->factors()) {
      f.second->draw(_canvas);
    }

    _canvas->flush();
  }
} // namespace srrg2_solver_gui