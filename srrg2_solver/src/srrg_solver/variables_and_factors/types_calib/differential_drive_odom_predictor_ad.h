#pragma once
#include <srrg_boss/serializable.h>
#include <srrg_geometry/ad.h>
#include <srrg_geometry/geometry_defs.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  /**
   * @brief Kinematic model of a differential drive robot.
   * It is used to predict the robot motion given the encoder readings
   */
  class DifferentialDriveOdomPredictorAD {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief set the encoder readings [l,r]
     * @param[in] Vector2f left and right encoder reading got in a time interval
     */
    inline void setTicks(const Vector2f& ticks_) {
      _ticks = ticks_;
      convertMatrix(_ad_ticks, ticks_);
    }

    /**
     * @brief get the encoder readings [l,r]
     * @return Vector2f left and right encoder reading got in a time interval
     */
    inline const Vector2f& ticks() {
      return _ticks;
    }

  protected:
    /**
     * @brief predicts the robot motion given the ticks and the parameters
     * @param[in] Vector3AD estimated kinematic parameters of the robot
     * @return Vector3AD predicted robot motion
     */
    Vector3_<DualValuef> _predictRobotMotion(const Vector3_<DualValuef>& dd_params_);

    /**
     * @brief use taylor expansion to compute some trigonometric functions for motion comutation
     * @param[out] DualValuef sin_theta_over_theta: taylor expansion value from sin(th)/th
     * @param[out] DualValuef one_minus_cos_theta_over_theta: taylor expansion value from
     *             (1-cos(th))/th
     * @param[in] DualValuef theta: angle driven by the robot wrt the x-axis
     */
    inline void _computeThetaTerms(DualValuef& sin_theta_over_theta,
                                   DualValuef& one_minus_cos_theta_over_theta,
                                   const DualValuef& theta);
    static const DualValuef cos_coeffs[]; /**< taylor expansion's cos coefficients*/
    static const DualValuef sin_coeffs[]; /**< taylor expansion's sin coefficients*/
    Vector2_<DualValuef> _ad_ticks;       /**< stored autodiff ticks*/
    Vector2f _ticks;                      /**< stored ticks*/
  };

} // namespace srrg2_solver
