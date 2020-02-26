#include "differential_drive_odom_predictor_ad.h"
#include <srrg_boss/object_data.h>
namespace srrg2_solver {

  Vector3_<DualValuef>
  DifferentialDriveOdomPredictorAD::_predictRobotMotion(const Vector3_<DualValuef>& dd_params_) {
    const DualValuef delta_l(_ad_ticks[0] * dd_params_[0]);
    const DualValuef delta_r(_ad_ticks[1] * dd_params_[1]);
    const DualValuef& baseline(dd_params_[2]);

    const DualValuef delta_plus(delta_r + delta_l);
    const DualValuef delta_minus(delta_r - delta_l);

    // dth
    const DualValuef dt = (delta_minus / baseline);

    DualValuef one_minus_cos_theta_over_theta, sin_theta_over_theta;
    _computeThetaTerms(sin_theta_over_theta, one_minus_cos_theta_over_theta, dt);

    const DualValuef dx = DualValuef(0.5) * delta_plus * sin_theta_over_theta;
    const DualValuef dy = DualValuef(0.5) * delta_plus * one_minus_cos_theta_over_theta;

    return Vector3_<DualValuef>(dx, dy, dt);
  }

  inline void DifferentialDriveOdomPredictorAD::_computeThetaTerms(DualValuef& sin_theta_over_theta,
                                                  DualValuef& one_minus_cos_theta_over_theta,
                                                  const DualValuef& theta) {
    // evaluates the taylor expansion of sin(x)/x and (1-cos(x))/x,
    // where the linearization point is x=0, and the functions are evaluated
    // in x=theta
    sin_theta_over_theta           = DualValuef(0);
    one_minus_cos_theta_over_theta = DualValuef(0);

    DualValuef theta_acc(1);
    DualValuef theta_acc_prev(1);
    for (uint8_t i = 0; i < 6; i++) {
      if (i & 0x1) {
        one_minus_cos_theta_over_theta += DualValuef(cos_coeffs[i]) * theta_acc;
      } else {
        sin_theta_over_theta += DualValuef(sin_coeffs[i]) * theta_acc;
      }
      theta_acc_prev = theta_acc;
      theta_acc *= theta;
    }
  }

  const DualValuef DifferentialDriveOdomPredictorAD::cos_coeffs[] = {DualValuef(0.),
                                                    DualValuef(0.5),
                                                    DualValuef(0.),
                                                    DualValuef(-1.0 / 24.0),
                                                    DualValuef(0.),
                                                    DualValuef(1.0 / 720.0)};

  const DualValuef DifferentialDriveOdomPredictorAD::sin_coeffs[] = {DualValuef(1.),
                                                    DualValuef(0.),
                                                    DualValuef(-1. / 6.),
                                                    DualValuef(0.),
                                                    DualValuef(1. / 120),
                                                    DualValuef(0.)};

} // namespace srrg2_solver
