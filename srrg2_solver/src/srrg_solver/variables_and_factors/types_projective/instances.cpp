#include "instances.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  void projective_registerTypes() {
    BOSS_REGISTER_CLASS(SE3ProjectiveErrorFactor);
    BOSS_REGISTER_CLASS(SE3ProjectiveDepthErrorFactor);
    BOSS_REGISTER_CLASS(SE3RectifiedStereoProjectiveErrorFactor);
    BOSS_REGISTER_CLASS(SE3ProjectiveErrorFactorCorrespondenceDriven);
    BOSS_REGISTER_CLASS(SE3ProjectiveDepthErrorFactorCorrespondenceDriven);
    BOSS_REGISTER_CLASS(SE3RectifiedStereoProjectiveErrorFactorCorrespondenceDriven);
  }
} // namespace srrg2_solver
