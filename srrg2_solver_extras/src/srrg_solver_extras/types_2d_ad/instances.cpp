#include "instances.h"
#include "se2_point2point_left_error_factor_ad.h"
#include "se2_pose_point_bearing_left_error_factor_ad.h"
#include "se2_pose_point_left_error_factor_ad.h"
#include "se2_pose_pose_left_error_factor_ad.h"

#include <srrg_solver/solver_core/solvers_all_impl.cpp>
#include <srrg_solver/solver_core/instance_macros.h>

namespace srrg2_solver_extras {
  void registerTypes2DAutoDiff() {
    BOSS_REGISTER_CLASS(SE2Point2PointLeftErrorFactorAD);
    BOSS_REGISTER_CLASS(SE2Point2PointLeftErrorFactorCorrespondenceDrivenAD);
    BOSS_REGISTER_CLASS(SE2PosePointBearingLeftErrorFactorAD);
    BOSS_REGISTER_CLASS(SE2PosePointLeftErrorFactorAD);
    BOSS_REGISTER_CLASS(SE2PosePoseLeftErrorFactorAD);
  }
} // namespace srrg2_solver_extras