#include "iteration_algorithm_dl.h"

namespace srrg2_solver {
  using namespace std;
  using namespace srrg2_core;

  void IterationAlgorithmDL::setSolver(SolverBase* solver_) {
    IterationAlgorithmBase::setSolver(solver_);
    _radius = param_initial_radius.value();
    _lambda = param_min_lambda.value();
  }

  void IterationAlgorithmDL::_scaleVector(std::vector<float>& v, const float& scale) const {
    int size = v.size();
    for (int i = 0; i < size; ++i) {
      v[i] *= scale;
    }
  }

  void IterationAlgorithmDL::_updateRadius(const float& chi_ratio) {
    if (chi_ratio == 0.f) {
      return;
    } else if (chi_ratio > param_min_ratio_up.value()) {
      _radius = std::max(_radius, 3 * _dogleg_step_norm);
    } else if (chi_ratio < param_max_ratio_down.value()) {
      _radius *= 0.5;
    }
  }

  void IterationAlgorithmDL::_computeTrustRegionScales() {
    // the ellipsoidal trust region is determined by the square-root entries of
    // the hessian matrix diagonal
    getDiagonal(_trust_region_scales);
    // we clip the minimum of the entries to avoid division by zero
    float min_diag = param_min_diag.value();
    for (size_t i = 0; i < _trust_region_scales.size(); ++i) {
      _trust_region_scales[i] = std::sqrt(std::max(_trust_region_scales[i], min_diag));
    }
  }

  void IterationAlgorithmDL::_trustRegionScaling(std::vector<float>& v) const {
    assert(v.size() == _trust_region_scales.size() &&
           "IterationAlgorithmDL::_trust_region_scales| mismatch size vector");
    for (size_t i = 0; i < v.size(); ++i) {
      v[i] /= _trust_region_scales[i];
    }
  }

  void IterationAlgorithmDL::_trustRegionUnscaling(std::vector<float>& v) const {
    assert(v.size() == _trust_region_scales.size() &&
           "IterationAlgorithmDL::_trust_region_scales| mismatch size vector");
    for (size_t i = 0; i < v.size(); ++i) {
      v[i] *= _trust_region_scales[i];
    }
  }

  void IterationAlgorithmDL::_computeCauchyStep() {
    // get negative gradient step
    getRHS(_cauchy_step);
    float squared_norm = 0.f;
    std::vector<float> scaled_cauchy(_cauchy_step.size(), 0);
    for (size_t i = 0; i < _cauchy_step.size(); ++i) {
      // scale the gradient according to the trust region scales
      const float& scale = _trust_region_scales[i];
      _cauchy_step[i] /= scale;
      // compute squared norm of the step
      squared_norm += _cauchy_step[i] * _cauchy_step[i];
      // scale the gradient again to properly compute || J * D^-1 * D^-1 * b ||^2 where D are the
      // trust region scales. In fact
      // || J * D^-1 * D^-1 * b ||^2 = (D^-1 * D^-1 * b)^T * H * (D^-1 * D^-1 * b)
      scaled_cauchy[i] = _cauchy_step[i] / scale;
    }
    _gradient_norm = std::sqrt(squared_norm);
    float gHg      = IterationAlgorithmBase::HessianSymmetricProduct(scaled_cauchy);
    // alpha factor of the cauchy step (determine the minimum of the linear model in the gradient
    // direction)
    _alpha         = _gradient_norm * _gradient_norm / gHg;
    _scaleVector(_cauchy_step, _alpha);
  }

  bool IterationAlgorithmDL::_computeGaussNewtonStep(IterationStats& istat) {
    // we solve a damped version of GN.
    // In particular we seek for the minimum value of the damping that makes the H matrix positive
    // definite
    _gn_step.clear();
    // get initial diagonal
    std::vector<float> diagonal;
    getDiagonal(diagonal);
    std::vector<float> current_diag(diagonal.size(), 0.f);
    while (_lambda <= param_max_lambda.value()) {
      // dump diagonal according to the current value of lambda
      for (size_t i = 0; i < diagonal.size(); ++i) {
        current_diag[i] = diagonal[i] + _lambda;
      }
      setDiagonal(current_diag);
      // solve GN step
      if (solveQuadraticForm(istat)) {
        getPerturbation(_gn_step);
        break;
      }
      // if fail increase the damping
      _lambda *= param_lambda_factor.value();
    }
    // if GN fails dogleg fail
    if (_gn_step.empty()) {
      return false;
    }
    float squared_norm = 0.f;
    for (size_t i = 0; i < _gn_step.size(); ++i) {
      // scale the GN step according to the trust region
      // in this case the scaling is applied by multiplying, in fact
      //   dx = - (D^-1 J^T J D^-1)^-1 (D^-1 b)
      //      = - D (J^T J)^-1 D D^-1 b
      //      = - D (J^T J)^-1 b
      //  where D are the trust region scales
      const float& scale = _trust_region_scales[i];
      _gn_step[i] *= scale;
      // compute squared norm
      squared_norm += _gn_step[i] * _gn_step[i];
    }
    _gn_step_norm = std::sqrt(squared_norm);
    return true;
  }

  void IterationAlgorithmDL::_computeStep(float& linear_decrease, const float& current_chi) {
    // clipped norm of the gradient in the case of a cauchy step
    float clipped_gradient_norm = 0;
    // compute useful quantities to determine the dogleg step
    float cauchy_norm       = _alpha * _gradient_norm;
    // discriminant is used to determine the mixing factor between gradient
    // step and Gn step
    float discriminant            = 0.f;
    // squared norm difference betwenn GN and GD steps
    float squared_norm_difference = 0.f;
    for (size_t i = 0; i < _gn_step.size(); ++i) {
      float diff              = _gn_step[i] - _cauchy_step[i];
      squared_norm_difference += diff * diff;
      discriminant += _cauchy_step[i] * diff;
    }
    if (_gn_step_norm <= _radius) {
      // scale gauss newton step according to the trust region ellipsoid
      // in this case the step come back to be the traditional GN step
      _trustRegionScaling(_gn_step);
      setPerturbation(_gn_step);
      // unscale for future applications of the step
      _trustRegionUnscaling(_gn_step);
      _dogleg_step_norm = _gn_step_norm;
      // the linear descrease in GN equals the current chi
      linear_decrease = current_chi;
    } else if (cauchy_norm >= _radius) {
      // linear decrease in the gradient direction
      linear_decrease = _radius * (2 * cauchy_norm - _radius) / (2 * _alpha);
      // makes the norm of the gradient equal to the trust region radius
      clipped_gradient_norm = _radius / cauchy_norm;
      _scaleVector(_cauchy_step, clipped_gradient_norm);
      // scale the step according to the ellipsoidal trust region
      _trustRegionScaling(_cauchy_step);
      setPerturbation(_cauchy_step);
      // unscale step for future usage
      _trustRegionUnscaling(_cauchy_step);
      _scaleVector(_cauchy_step, 1.f / clipped_gradient_norm);
      // norm of the step equals the radius
      _dogleg_step_norm = _radius;
    } else {
      // determine the beta parameter according to the formula in
      // http://www2.imm.dtu.dk/pubdb/views/edoc_download.php/3215/pdf/imm3215.pdf
      float radius_minus_cauchy = _radius * _radius - cauchy_norm * cauchy_norm;
      float sqrt_term =
        std::sqrt(discriminant * discriminant + squared_norm_difference * radius_minus_cauchy);
      float beta = 0;
      if (discriminant <= 0) {
        beta = (-discriminant + sqrt_term) / squared_norm_difference;
      } else {
        beta = radius_minus_cauchy / (discriminant + sqrt_term);
      }
      // linear decrease for the dogleg step
      linear_decrease = 0.5 * _alpha * (1 - beta) * (1 - beta) * _gradient_norm * _gradient_norm +
                        beta * (2 - beta) * current_chi;
      // compute the dogleg step and its norm
      std::vector<float> dogleg_step(_cauchy_step.size(), 0);
      float squared_norm = 0;
      for (size_t i = 0; i < dogleg_step.size(); ++i) {
        dogleg_step[i] = _cauchy_step[i] + beta * (_gn_step[i] - _cauchy_step[i]);
        squared_norm += dogleg_step[i] * dogleg_step[i];
      }
      _dogleg_step_norm = std::sqrt(squared_norm);
      // scale the step according to the ellipsoidal trust region
      _trustRegionScaling(dogleg_step);
      setPerturbation(dogleg_step);
    }
  }

  bool IterationAlgorithmDL::oneRound() {
    IterationStats istat;
    istat.reset();
    istat.iteration = currentIteration();
    if (!buildQuadraticForm(istat)) {
      iterationStats().push_back(istat);
      return false;
    }
    // compute the trust region scales in each direction
    // we use an ellipsoidal trust region model
    _computeTrustRegionScales();
    _computeCauchyStep();
    if (!_computeGaussNewtonStep(istat)) {
      return false;
    }
    // save chi
    float current_chi = 0.5 * istat.chi_normalized;
    // linear_decrease : the chi decrease according to the approximate model on the GN parabola
    float linear_decrease = current_chi;
    // initialize iteration and chi ratio
    int iter    = 0;
    float ratio = 0;
    do {
      push();
      // compute step according to the dogleg formula
      _computeStep(linear_decrease, current_chi);
      // apply perturbation and compute ratio between the actual chi decrease
      // and the one predicted my the linear model
      applyPerturbation(istat);
      updateChi(istat);
      ratio = (current_chi - 0.5 * istat.chi_normalized) / linear_decrease;
      // update radius according to the ratio
      _updateRadius(ratio);
      ++iter;
      if (ratio > 0) {
        // accept the step
        discardTop();
        // in this case we put in the lambda of the iteration stats the radius of the trust region
        istat.lambda = _radius;
        // update the damping for the next solving of the GN step (make it lower)
        // we assume to be out of singularities after a good step
        float min_lambda    = param_min_lambda.value();
        float lambda_factor = param_lambda_factor.value();
        _lambda                      = std::max(min_lambda, 2.f * _lambda / lambda_factor);
        istat.num_internal_iteration = iter;
        iterationStats().push_back(istat);
        return true;
      } else if (ratio < 0) {
        pop();
      } else {
        break;
      }
    } while (iter < param_max_iterations.value());
    istat.lambda                 = _radius;
    istat.num_internal_iteration = iter;
    iterationStats().push_back(istat);
    return (ratio > 0);
  }
} // namespace srrg2_solver
