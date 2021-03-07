#include "se2_pose_pose_left_error_factor_ad.h"
#include <srrg_geometry/geometry3d.h>

//! include this: this contains all the implementations of the factors
//! that are hidden to the modules that do not need them to avoid excessive compilation times (EVIL)
#include <srrg_solver/solver_core/ad_error_factor_impl.cpp>
#include <srrg_solver/solver_core/error_factor_impl.cpp>
#include <srrg_solver/solver_core/instance_macros.h>


namespace srrg2_solver_extras {
  using namespace srrg2_core;
  using namespace srrg2_solver;
  void SE2PosePoseLeftErrorFactorAD::_drawImpl(srrg2_core::ViewerCanvasPtr canvas) const {
    if (!canvas) {
      return;
    }

    Vector3f segments[2];
    const VariableSE2RightAD* from_v = dynamic_cast<const VariableSE2RightAD*>(variable(0));
    const VariableSE2RightAD* to_v   = dynamic_cast<const VariableSE2RightAD*>(variable(1));
    if (!from_v || !to_v) {
      return;
    }

    const Isometry2f& from = from_v->estimate();
    const Isometry2f& to   = to_v->estimate();
    segments[0] << from.translation().x(), from.translation().y(), 0.f;
    segments[1] << to.translation().x(), to.translation().y(), 0.f;
    canvas->putSegment(2, segments, 0);
  }

  INSTANTIATE(SE2PosePoseLeftErrorFactorAD)

} // namespace srrg2_solver
