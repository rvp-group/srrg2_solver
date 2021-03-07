#include "instances.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  void variables_and_factors_projective_registerTypes() {
    BOSS_REGISTER_CLASS(SE3ProjectiveErrorFactor);
    BOSS_REGISTER_CLASS(SE3ProjectiveDepthErrorFactor);
    BOSS_REGISTER_CLASS(SE3RectifiedStereoProjectiveErrorFactor);
    BOSS_REGISTER_CLASS(SE3ProjectiveErrorFactorCorrespondenceDriven);
    BOSS_REGISTER_CLASS(SE3ProjectiveDepthErrorFactorCorrespondenceDriven);
    BOSS_REGISTER_CLASS(SE3RectifiedStereoProjectiveErrorFactorCorrespondenceDriven);

    // ldg serialization similiarity variables and factors
    BOSS_REGISTER_CLASS(VariableSim3QuaternionRight);
    BOSS_REGISTER_CLASS(VariableSim3QuaternionRightAD);
    BOSS_REGISTER_CLASS(VariableSim3QuaternionLeft);
    BOSS_REGISTER_CLASS(VariableSim3QuaternionLeftAD);
    BOSS_REGISTER_CLASS(VariableSim3EulerRight);
    BOSS_REGISTER_CLASS(VariableSim3EulerRightAD);
    BOSS_REGISTER_CLASS(VariableSim3EulerLeft);
    BOSS_REGISTER_CLASS(VariableSim3EulerLeftAD);
    BOSS_REGISTER_CLASS(Sim3PosePoseErrorFactorAD);
    BOSS_REGISTER_CLASS(Sim3Point2PointErrorFactor);
    BOSS_REGISTER_CLASS(SE3PosePointOmniBAErrorFactor);
  }

} // namespace srrg2_solver
