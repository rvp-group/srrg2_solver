#include "srrg_solver/variables_and_factors/types_2d/instances.h"
#include "instances.h"
#include "srrg_solver/solver_core/factor_impl.cpp"
#include "srrg_solver/variables_and_factors/types_3d/instances.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  // this is the function you have to call to initialize
  // the serialization subsystem
  void variables_and_factors_calib_registerTypes() {
    BOSS_REGISTER_CLASS(DifferentialDriveOdomErrorFactorAD);
    BOSS_REGISTER_CLASS(DifferentialDriveOdomSensor2DErrorFactorAD);
    BOSS_REGISTER_CLASS(DifferentialDriveOdomSensor3DErrorFactorAD);
    BOSS_REGISTER_CLASS(VariableTime);
    BOSS_REGISTER_CLASS(VariableTimeAD);
    BOSS_REGISTER_CLASS(SE3SensorPoseTimeDelayErrorFactorAD);
    BOSS_REGISTER_CLASS(DifferentialDriveOdomTimeDelaySensor2DErrorFactorAD);
    BOSS_REGISTER_CLASS(SE2SensorExtrinsicPoseMotionCalibAD);
    BOSS_REGISTER_CLASS(SE3SensorExtrinsicPoseMotionCalibAD);
  }
} // namespace srrg2_solver
