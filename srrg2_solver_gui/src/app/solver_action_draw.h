#pragma once
#include <srrg_solver/solver_core/factor_graph.h>
#include <srrg_solver/solver_core/solver_action_base.h>
#include <srrg_solver/solver_core/solver_base.h>
#include <srrg_viewer/viewer_canvas.h>

namespace srrg2_solver_gui {

  //! @brief action that draws a factor graph on the canvas
  class SolverActionDraw : public srrg2_solver::SolverActionBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    using BaseType = srrg2_solver::SolverActionBase;

    SolverActionDraw() = delete;
    SolverActionDraw(srrg2_solver::SolverBase* solver_,
                     const srrg2_core::ViewerCanvasPtr& canvas_ptr_,
                     const srrg2_solver::FactorGraphPtr& graph_ptr_) :
      BaseType(solver_) {
      _canvas = canvas_ptr_;
      _graph  = graph_ptr_;
    }

    virtual ~SolverActionDraw() {
    }

    void doAction() final;

  protected:
    srrg2_core::ViewerCanvasPtr _canvas = nullptr;
    srrg2_solver::FactorGraphPtr _graph = nullptr;
  };

  using SolverActionDrawPtr = std::shared_ptr<SolverActionDraw>;
} // namespace srrg2_solver_gui