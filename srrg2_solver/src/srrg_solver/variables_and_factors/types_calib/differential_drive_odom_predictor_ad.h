#pragma once
#include <srrg_boss/serializable.h>
#include <srrg_geometry/ad.h>
#include <srrg_geometry/geometry_defs.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  class DifferentialDriveOdomPredictorAD{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    inline void setTicks(const Vector2f& ticks_) {
      _ticks = ticks_;
      convertMatrix(_ad_ticks, ticks_);
    }

    inline const Vector2f ticks() {
      return _ticks;
    }

  protected:
    Vector3_<DualValuef> _predictRobotMotion(const Vector3_<DualValuef>& dd_params_);
    inline void _computeThetaTerms(DualValuef& sin_theta_over_theta,
                                   DualValuef& one_minus_cos_theta_over_theta,
                                   const DualValuef& theta);
    static const DualValuef cos_coeffs[];
    static const DualValuef sin_coeffs[];
    Vector2_<DualValuef> _ad_ticks;
    Vector2f _ticks;
  };

} // namespace srrg2_solver
