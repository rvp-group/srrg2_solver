#include "termination_criteria.h"
#include "solver_base.h"

namespace srrg2_solver {

  void TerminationCriteria::getRHS(std::vector<float>& b_) const {
    if(_solver){
      _solver->getRHS(b_);
    }
  }

  void TerminationCriteria::getPerturbation(std::vector<float>& dx_) const {
    if(_solver){
      _solver->getPerturbation(dx_);
    }
  }

  bool SimpleTerminationCriteria::hasToStop() const {
    // 1st iteration always good
    int current_iteration = _solver->currentIteration();
    if (current_iteration<2) {
      return false;
    }
    const IterationStats& prev_stats = _solver->iterationStats()[current_iteration-2];
    const IterationStats& curr_stats = _solver->iterationStats()[current_iteration-1];

    // number of inliers should be steady
    if (prev_stats.num_inliers != curr_stats.num_inliers) {
      return false;
    }

    
    // number of outliers should be steady
    if (prev_stats.num_outliers != curr_stats.num_outliers) {
      return false;
    }

    // inlier error steady
    if (prev_stats.chi_inliers > 0) {
      float inlier_chi_ratio = 1.f - curr_stats.chi_inliers / prev_stats.chi_inliers;
      if (inlier_chi_ratio > param_epsilon.value()) {
        return false;
      }
    }

    // outlier error steady
    if (prev_stats.chi_outliers > 0) {
      // chi of outliers should be steady
      float outlier_chi_ratio = 1.f - curr_stats.chi_outliers / prev_stats.chi_outliers;
      if (outlier_chi_ratio > param_epsilon.value()) {
        return false;
      }
    }

    // ok, all conditions are met, we stop
    return true;
  }

  bool PerturbationNormTerminationCriteria::hasToStop() const {
    std::vector<float> perturbation;
    getPerturbation(perturbation);
    float squared_norm = 0;
    for (const float& x_i : perturbation) {
      squared_norm += x_i*x_i;
    }

    if(std::sqrt(squared_norm) > param_epsilon.value()){
      return false;
    }

    return true;
  }

  bool RelativeGradientChiTerminationCriteria::hasToStop() const {
    std::vector<float> gradient;
    getRHS(gradient);
    float squared_norm_gradient = 0;
    for (const float& x_i : gradient) {
      squared_norm_gradient += x_i * x_i;
    }
    float stabilized_error_norm = 1 + std::sqrt(_solver->iterationStats().back().chi_normalized);
    float gradient_norm         = std::sqrt(squared_norm_gradient);
    if (gradient_norm > param_epsilon.value() * stabilized_error_norm) {
      return false;
    }
    return true;
  }

}
