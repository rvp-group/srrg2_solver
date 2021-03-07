#pragma once
#include <srrg_property/property.h>

#include "iteration_algorithm_base.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  /*! @brief DogLeg optimization algorithm with ellipsoidal trust region*/
  class IterationAlgorithmDL : public IterationAlgorithmBase {
  public:
    PARAM(PropertyFloat,
          initial_radius,
          "initial radius of the trust region [default: 1e4]",
          1e4,
          0);
    PARAM(PropertyFloat,
          min_ratio_up,
          "min value of the chi ratio to increase the radius of the trust region",
          0.75,
          0);
    PARAM(PropertyFloat,
          max_ratio_down,
          "max value of the chi ratio to decrease the radius of the trust region",
          0.25,
          0);
    PARAM(PropertyInt, max_iterations, "max iterations of the algorithm[default: 20]", 20, 0);
    PARAM(PropertyFloat, min_lambda, "Minimum value for the damping of the GN step", 1e-8, 0);
    PARAM(PropertyFloat, max_lambda, "Maximum value for the damping of the GN step", 1.0, 0);
    PARAM(PropertyFloat, lambda_factor, "Increase factor of the damping in the GN step", 10.0, 0);
    PARAM(PropertyFloat,
          min_diag,
          "Minimum diagonal value for the H to compute the trust region scales",
          1e-6,
          0);

    void setSolver(SolverBase* _solver) final;
    bool oneRound() final;

  protected:
    /*! radius of the trust region, damping of the GN step and magnitude of the cauchy step */
    float _radius = 0, _lambda = 0, _alpha = 0;
    /*! steps norm */
    float _gradient_norm = 0, _gn_step_norm = 0, _dogleg_step_norm = 0;
    /*! Scale vector according to a scalar value
     * @param[in/out] v the vector
     * @param[in] scale the scalar
     */
    void _scaleVector(std::vector<float>& v, const float& scale) const;
    /*! Scale a vector according to the trust region scales */
    void _trustRegionScaling(std::vector<float>& v) const;
    /*! Unscale a vector according to the trust region scales */
    void _trustRegionUnscaling(std::vector<float>& v) const;
    /*! Update the trust region according to the ratio between the actual chi decrease and the
     * predicted one by the linear model
     *  @param[in] the chi ratio
     * */
    void _updateRadius(const float& chi_ratio);
    /*! Compute the cauchy step, which minimize the linear model in the gradient direction */
    void _computeCauchyStep();
    /*! Determine the gauss-newton step using the minimum value of the damping that makes the H
     * matrix non singular*/
    bool _computeGaussNewtonStep(IterationStats& istat);
    /*! Compute the Dogleg step (virtual for the DoubleDogLeg step)
     *  @param[in] the current chi square value
     *  @param[out] the predicted chi decrease using the linear model
     * */
    virtual void _computeStep(float& linear_decrease, const float& current_chi);
    /*! Compute the scales that determine the elliptical trust region */
    void _computeTrustRegionScales();
    /*! store steps */
    std::vector<float> _gn_step, _cauchy_step, _trust_region_scales;
  };

} // namespace srrg2_solver
