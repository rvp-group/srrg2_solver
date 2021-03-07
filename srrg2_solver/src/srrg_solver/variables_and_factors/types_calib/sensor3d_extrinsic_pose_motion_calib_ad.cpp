#include "sensor3d_extrinsic_pose_motion_calib_ad.h"
#include "srrg_solver/solver_core/ad_error_factor_impl.cpp"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  void SE3SensorExtrinsicPoseMotionCalibAD::serialize(ObjectData& odata, IdContext& context) {
    BaseType::serialize(odata, context);
    odata.setEigen("from", _from);
    odata.setEigen("to", _to);
  }

  void SE3SensorExtrinsicPoseMotionCalibAD::deserialize(ObjectData& odata, IdContext& context) {
    BaseType::deserialize(odata, context);
    _from = odata.getEigen<Isometry3f>("from");
    _to   = odata.getEigen<Isometry3f>("to");
    setFrom(_from);
    setTo(_to);
  }

  INSTANTIATE(SE3SensorExtrinsicPoseMotionCalibAD)
} // namespace srrg2_solver
