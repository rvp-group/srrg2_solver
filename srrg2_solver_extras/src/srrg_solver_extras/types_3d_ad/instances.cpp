#include "instances.h"

#include "se3_point2point_euler_error_factor_ad.h"
#include "se3_pose_point_euler_error_factor_ad.h"
#include "se3_pose_point_offset_euler_error_factor_ad.h"
#include "se3_pose_pose_chordal_quaternion_error_factor_ad.h"
#include "se3_pose_pose_error_factor_ad.h"
#include "se3_pose_pose_euler_error_factor_ad.h"
#include "se3_prior_euler_error_factor_ad.h"

namespace srrg2_solver_extras{
  void registerTypes3DAutoDiff(){
    BOSS_REGISTER_CLASS(SE3Point2PointEulerErrorFactorAD);
    BOSS_REGISTER_CLASS(SE3Point2PointEulerErrorFactorCorrespondenceDrivenAD);
    BOSS_REGISTER_CLASS(SE3PosePointEulerErrorFactorAD);
    BOSS_REGISTER_CLASS(SE3PosePointOffsetEulerErrorFactorAD);
    BOSS_REGISTER_CLASS(SE3PosePoseChordalQuaternionErrorFactorAD);
    BOSS_REGISTER_CLASS(SE3PosePoseErrorFactorAD);
    BOSS_REGISTER_CLASS(SE3PosePoseEulerErrorFactorAD);
    BOSS_REGISTER_CLASS(SE3PriorEulerErrorFactorAD);
  }
}