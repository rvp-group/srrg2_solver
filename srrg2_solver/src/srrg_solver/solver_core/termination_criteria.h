#pragma once
#include <srrg_config/configurable.h>
#include <srrg_property/property.h>

#include "solver_stats.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  class SolverBase;
  /*! @brief Base termination criteria interface, do nothing.
    In the derived class you must override the hasToStop() method */
  class TerminationCriteria : public Configurable {
  public:
    /*! @return false if optimization have to stop */
    virtual bool hasToStop() const {
      return false;
    }
    /*! Set the pointer to the solver at which the termination criteria is attached
     @param[in] solver_ pointer to the solver
     */
    void setSolver(const SolverBase* solver_) {
      _solver = solver_;
    }

  protected:
    /*! Call the corresponding function in SolverBase */
    void getRHS(std::vector<float>& b_) const;
    /*! Call the corresponding function in SolverBase */
    void getPerturbation(std::vector<float>& dx_) const;
    const SolverBase* _solver = 0; /*!<Pointer to the solver
                                     at which the termination criteria is
                                     attached */
  };

  using TerminationCriteriaPtr = std::shared_ptr<TerminationCriteria>;

  /*! @brief Termination criteria that monitors the evolution of chi2
   and number of outliers/inliers
   inliers/outliers should be stable
   their errors as well */
  class SimpleTerminationCriteria : public TerminationCriteria {
  public:
    PARAM(PropertyFloat, epsilon, "ratio of decay of chi2 between iteration", 1e-3, 0);
    bool hasToStop() const override;
  };

  /*! @brief Gradient norm based termination criteria, stop if the norm of the gradient is below the
   * threshold */
  class GradientNormTerminationCriteria : public TerminationCriteria {
  public:
    PARAM(PropertyFloat, epsilon, "maximum gradient norm", 1e-5, 0);
    bool hasToStop() const override;
  };

  /*! @brief Perturbation norm based termination criteria, stop if the norm of the perturbation
   (solution of the linearized system) is below the threshold */
  class PerturbationNormTerminationCriteria : public TerminationCriteria {
  public:
    PARAM(PropertyFloat, epsilon, "maximum perturbation norm", 1e-5, 0);
    bool hasToStop() const override;
  };

} // namespace srrg2_solver
