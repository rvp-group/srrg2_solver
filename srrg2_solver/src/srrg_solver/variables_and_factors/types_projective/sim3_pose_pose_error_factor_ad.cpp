#include "sim3_pose_pose_error_factor_ad.h"
#include "srrg_solver/solver_core/ad_error_factor_impl.cpp"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {

  void Sim3PosePoseErrorFactorAD::_drawImpl(ViewerCanvasPtr canvas_) const {
    if (!canvas_)
      throw std::runtime_error("Sim3PosePoseErrorFactorAD::draw|invalid canvas");
    Vector3f factor_line[2];
    const VariableSim3QuaternionRight* v_from=dynamic_cast<const VariableSim3QuaternionRight*>(variable(0));
    const VariableSim3QuaternionRight* v_to=dynamic_cast<const VariableSim3QuaternionRight*>(variable(1));
    if (! v_from || ! v_to) {
      return;
    }
    const Similiarity3f& pose_from = v_from->estimate();
    const Similiarity3f& pose_to =   v_to->estimate();

    factor_line[0] = pose_from.translation()/pose_from.inverseScaling();
    factor_line[1] = pose_to.translation()/pose_from.inverseScaling();

    canvas_->pushColor();
    canvas_->setColor(srrg2_core::ColorPalette::color3fOrange());
    canvas_->putLine(2, factor_line);

    canvas_->popAttribute();
  }

  INSTANTIATE(Sim3PosePoseErrorFactorAD)
} // namespace srrg2_solver
