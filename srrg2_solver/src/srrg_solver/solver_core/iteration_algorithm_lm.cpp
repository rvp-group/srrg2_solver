#include "iteration_algorithm_lm.h"

namespace srrg2_solver {
  using namespace std;
  using namespace srrg2_core;

  void IterationAlgorithmLM::setSolver(SolverBase* solver_) {
    IterationAlgorithmBase::setSolver(solver_);
    if (_solver) {
      getDiagonal(_diagonal);
      _dx.resize(_diagonal.size());
      _b.resize(_diagonal.size());
      _current_diagonal.resize(_diagonal.size());
    }
  }

  void IterationAlgorithmLM::cropScale(float& value) {
    value = std::min(value, param_step_high.value());
    value = std::max(value, param_step_low.value());
  }

  float IterationAlgorithmLM::computeScale() {
    float scale = 0;
    assert(_dx.size() == _b.size() && "size mismatch");
    for (size_t i = 0; i < _dx.size(); i++) {
      scale += _dx[i] * (_lambda * _dx[i] + _b[i]);
    }
    return scale;
  }

  float IterationAlgorithmLM::computeLambdaInit() {
    if (param_user_lambda_init.value() > 0) {
      return param_user_lambda_init.value();
    }
    float max_diagonal = 0;
    for (size_t i = 0; i < _diagonal.size(); ++i) {
      max_diagonal = std::max(fabs(_diagonal[i]), max_diagonal);
    }
    return param_tau.value() * max_diagonal;
  }

  void IterationAlgorithmLM::updateDiagonal() {
    // update diagonal
    bool vd = param_variable_damping.value();
    for (size_t i = 0; i < _current_diagonal.size(); ++i) {
      if (vd) {
        _current_diagonal[i] = _diagonal[i] * (1. + _lambda);
      } else {
        _current_diagonal[i] = _diagonal[i] + _lambda;
      }
    }
    setDiagonal(_current_diagonal);
  }

  bool IterationAlgorithmLM::oneRound() {
    IterationStats istat;
    istat.reset();
    istat.iteration = currentIteration();
    if (!buildQuadraticForm(istat)) {
      iterationStats().push_back(istat);
      return false;
    }

    // save chi
    float current_chi = istat.chi_normalized;
    float temp_chi    = current_chi;

    // backup diagonal
    getDiagonal(_diagonal);
    _current_diagonal = _diagonal;

    // compute lambda init
    if (!iterationStats().size()) {
      _lambda = computeLambdaInit();
      _ni     = 2;
    }

    float chi_delta = 0;
    // iterate for lm, each time we add a thing to the stats
    int lm_iteration = 0;
    do {
      // save state
      push();
      updateDiagonal();

      istat.lambda = _lambda;
      // tg try to solver the system with the current value of lambda
      if (!solveQuadraticForm(istat)) {
        // if fail set actual chi to max
        temp_chi = std::numeric_limits<float>::max();
      } else {
        // else apply perturbation and save chi
        applyPerturbation(istat);
        updateChi(istat);
        temp_chi = istat.chi_normalized;
      }
      // compute scale
      getPerturbation(_dx);
      getRHS(_b);
      float scale = computeScale();
      scale += 1e-3;
      // tg compute actual chi variation
      chi_delta = (current_chi - temp_chi) / scale;
      // tg if the solution is good
      if (chi_delta > 0 && std::isfinite(current_chi)) {
        // tg compute actual reduction ratio for lambda
        float proposed_scale = 1. - pow((2 * chi_delta - 1), 3);
        // tg crop it between predefined values (see the params)
        cropScale(proposed_scale);
        // tg reset correction factor and scale lambda
        _ni = 2;
        _lambda *= proposed_scale;
        current_chi = temp_chi;
        discardTop();
        istat.num_internal_iteration = lm_iteration;
        iterationStats().push_back(istat);
        return true;
      } else {
        // tg apply lambda correction factor and increase it
        _lambda *= _ni;
        _ni *= 2;
        // tg remove current state
        pop();
        if (!std::isfinite(_lambda)) {
          break;
        }
      }
      ++lm_iteration;
    } while (chi_delta < 0 && lm_iteration < param_lm_iterations_max.value());

    setDiagonal(_diagonal);
    istat.num_internal_iteration = lm_iteration;
    // ia in this way we have a single stat for outer LM iter
    iterationStats().push_back(istat);

    if (chi_delta == 0) {
      return true;
    }
    if (!std::isfinite(_lambda) || lm_iteration == param_lm_iterations_max.value()) {
      return false;
    }
    return true;
  }
} // namespace srrg2_solver
