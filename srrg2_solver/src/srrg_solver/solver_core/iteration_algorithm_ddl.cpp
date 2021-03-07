#include "iteration_algorithm_ddl.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  void IterationAlgorithmDDL::_computeStep(float& linear_decrease, const float& current_chi) {
    // clipped norm of the step
    float clipped_step_norm = 0;
    // compute useful quantities to determine the double dogleg step
    float cauchy_norm = _alpha * _gradient_norm;
    float gn_step_projected_on_cauchy_step = 0.f;
    for (size_t i = 0; i < _cauchy_step.size(); ++i) {
      gn_step_projected_on_cauchy_step += _cauchy_step[i] * _gn_step[i];
    }
    // gamma = ||cauchy_step||^2/cauchy_step^T * gn_step
    float gamma = cauchy_norm * cauchy_norm / gn_step_projected_on_cauchy_step;
    // A = gamma^2 * ||gn_step||^2 - ||cauchy_step||^2
    float A     = gamma * gamma * _gn_step_norm * _gn_step_norm - cauchy_norm * cauchy_norm;
    if (cauchy_norm >= _radius) {
      // linear decrease in the gradient direction
      linear_decrease = _radius * (2 * cauchy_norm - _radius) / (2 * _alpha);
      // makes the norm of the gradient equal to the trust region radius
      clipped_step_norm = _radius / cauchy_norm;
      _scaleVector(_cauchy_step, clipped_step_norm);
      // scale the step according to the ellipsoidal trust region
      _trustRegionScaling(_cauchy_step);
      setPerturbation(_cauchy_step);
      // unscale step for future usage
      _trustRegionUnscaling(_cauchy_step);
      _scaleVector(_cauchy_step, 1.f / clipped_step_norm);
      // norm of the step equals the radius
      _dogleg_step_norm = _radius;
    } else if (gamma * _gn_step_norm >= _radius) {
      // B = radius^2 - ||cauchy_step|| ^2
      float B = _radius * _radius - cauchy_norm * cauchy_norm;
      // mixing factor between cauchy step and projected GN step
      float beta = (A - B) / (A + std::sqrt(A * B));
      // compute double-dog leg step and squared norm
      std::vector<float> dogleg_step(_cauchy_step.size(), 0);
      float squared_norm = 0;
      for (size_t i = 0; i < dogleg_step.size(); ++i) {
        dogleg_step[i] = beta * _cauchy_step[i] + (1 - beta) * gamma * _gn_step[i];
        squared_norm += dogleg_step[i] * dogleg_step[i];
      }
      _dogleg_step_norm = std::sqrt(squared_norm);
      // scale the step according to the ellipsoidal trust region
      _trustRegionScaling(dogleg_step);
      setPerturbation(dogleg_step);
      // linear decrease is a delirius, if you think do to better than that
      // simplify -dogleg_step^T * gradient - 0.5 * dogleg_step^T * H * dogleg_step
      // ... Now my formula seems nice right ? (:
      float one_minus_beta = 1 - beta;
      float gradient_term  = _alpha * beta * _gradient_norm * _gradient_norm;
      float gn_term        = one_minus_beta * gamma * current_chi;
      linear_decrease      = (1 - 0.5 * beta - 0.5 * one_minus_beta * gamma) * gradient_term;
      linear_decrease += (2 - gamma * one_minus_beta) * gn_term;
    } else if (_gn_step_norm >= _radius) {
      clipped_step_norm = _radius / _gn_step_norm;
      // linear decrease due to clipped newton step
      linear_decrease =
        (clipped_step_norm * clipped_step_norm + 0.5 * clipped_step_norm) * current_chi;
      // scale step norm
      _scaleVector(_gn_step, clipped_step_norm);
      // scale step according to ellipsoidal trust region
      _trustRegionScaling(_gn_step);
      setPerturbation(_gn_step);
      // unscale step for future usage
      _trustRegionUnscaling(_gn_step);
      _scaleVector(_gn_step, 1.f / clipped_step_norm);
      // norm of the step equals the radius
      _dogleg_step_norm = _radius;
    } else if (_gn_step_norm <= _radius) {
      // scale gauss newton step according to the trust region ellipsoid
      // in this case the step come back to be the traditional GN step
      _trustRegionScaling(_gn_step);
      setPerturbation(_gn_step);
      // unscale for future applications of the step
      _trustRegionUnscaling(_gn_step);
      _dogleg_step_norm = _gn_step_norm;
      // the linear descrease in GN equals the current chi
      linear_decrease = current_chi;
    }
  }
} // namespace srrg2_solver
