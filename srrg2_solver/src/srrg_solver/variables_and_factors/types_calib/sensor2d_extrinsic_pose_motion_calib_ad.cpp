#include "sensor2d_extrinsic_pose_motion_calib_ad.h"
#include "srrg_solver/solver_core/ad_error_factor_impl.cpp"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  void SE2SensorExtrinsicPoseMotionCalibAD::serialize(ObjectData& odata, IdContext& context) {
    BaseType::serialize(odata, context);
    odata.setEigen("from", _from);
    odata.setEigen("to", _to);
  }

  void SE2SensorExtrinsicPoseMotionCalibAD::deserialize(ObjectData& odata, IdContext& context) {
    BaseType::deserialize(odata, context);
    _from = odata.getEigen<Isometry2f>("from");
    _to   = odata.getEigen<Isometry2f>("to");
    setFrom(_from);
    setTo(_to);
  }

  INSTANTIATE(SE2SensorExtrinsicPoseMotionCalibAD)
} // namespace srrg2_solver
