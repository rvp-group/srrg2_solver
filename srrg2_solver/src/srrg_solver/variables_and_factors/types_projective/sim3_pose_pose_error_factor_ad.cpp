#include "sim3_pose_pose_error_factor_ad.h"
#include "srrg_solver/solver_core/ad_error_factor_impl.cpp"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {

  void Sim3PosePoseErrorFactorAD::draw(ViewerCanvasPtr canvas_) const {
    if (!canvas_)
      throw std::runtime_error("Sim3PosePoseErrorFactorAD::draw|invalid canvas");
    Vector3f factor_line[2];
    const Similiarity3f& pose_from =
      dynamic_cast<const VariableSim3EulerLeftAD*>(variable(0))->estimate();
    const Similiarity3f& pose_to =
      dynamic_cast<const VariableSim3EulerLeftAD*>(variable(1))->estimate();
    factor_line[0] = pose_from.translation();
    factor_line[1] = pose_to.translation();

    canvas_->pushColor();
    canvas_->setColor(srrg2_core::ColorPalette::color3fOrange());
    canvas_->putLine(2, factor_line);

    canvas_->popAttribute();
  }

  INSTANTIATE(Sim3PosePoseErrorFactorAD)
} // namespace srrg2_solver
