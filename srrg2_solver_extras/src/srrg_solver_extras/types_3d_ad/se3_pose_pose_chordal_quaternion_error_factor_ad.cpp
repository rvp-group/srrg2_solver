#include "se3_pose_pose_chordal_quaternion_error_factor_ad.h"
#include <srrg_solver/solver_core/ad_error_factor_impl.cpp>
#include <srrg_solver/solver_core/error_factor_impl.cpp>
#include <srrg_solver/solver_core/instance_macros.h>

namespace srrg2_solver_extras {

  void SE3PosePoseChordalQuaternionErrorFactorAD::_drawImpl(ViewerCanvasPtr canvas_) const {
    if (!canvas_)
      throw std::runtime_error("SE3PosePoseChordalQuaternionErrorFactorAD::draw|invalid canvas");
    Vector3f factor_line[2];
    const Isometry3f& pose_from =
      dynamic_cast<const VariableSE3EulerLeftAD*>(variable(0))->estimate();
    const Isometry3f& pose_to =
      dynamic_cast<const VariableSE3EulerLeftAD*>(variable(1))->estimate();
    factor_line[0] = pose_from.translation();
    factor_line[1] = pose_to.translation();

    canvas_->pushColor();
    canvas_->setColor(srrg2_core::ColorPalette::color3fOrange());
    canvas_->putLine(2, factor_line);

    canvas_->popAttribute();
  }

  INSTANTIATE(SE3PosePoseChordalQuaternionErrorFactorAD)
}
