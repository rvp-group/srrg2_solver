#pragma once
#include <srrg_config/configurable.h>
#include <srrg_property/property.h>

#include "solver_stats.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  class SolverBase;

  class TerminationCriteria : public Configurable {
  public:
    virtual bool hasToStop() const {
      return false;
    }

    virtual void setSolver(const SolverBase* solver_) {
      _solver = solver_;
    }

  protected:
    void getRHS(std::vector<float>& b_) const;
    void getPerturbation(std::vector<float>& dx_) const;
    const SolverBase* _solver = 0;
  };

  using TerminationCriteriaPtr = std::shared_ptr<TerminationCriteria>;

  // simplistic termination criteria that monitors the evolution of chi2
  // and number of outliers/inliers
  // inliers/outliers should be stable
  // their errors as well
  class SimpleTerminationCriteria : public TerminationCriteria {
  public:
    PARAM(PropertyFloat, epsilon, "ratio of decay of chi2 between iteration", 1e-3, 0);
    bool hasToStop() const override;
    void setSolver(const SolverBase* solver_) override {
      TerminationCriteria::setSolver(solver_);
    }
  };

  // @brief : Gradient norm based termination criteria (tg)

  class GradientNormTerminationCriteria : public TerminationCriteria {
  public:
    PARAM(PropertyFloat, epsilon, "maximum gradient norm", 1e-5, 0);
    bool hasToStop() const override;
    void setSolver(const SolverBase* solver_) override {
      TerminationCriteria::setSolver(solver_);
    }
  };

  // @brief : Perturbation norm based termination criteria (tg)

  class PerturbationNormTerminationCriteria : public TerminationCriteria {
  public:
    PARAM(PropertyFloat, epsilon, "maximum perturbation norm", 1e-5, 0);
    bool hasToStop() const override;
    void setSolver(const SolverBase* solver_) override {
      TerminationCriteria::setSolver(solver_);
    }
  };

} // namespace srrg2_solver
