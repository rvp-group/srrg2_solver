#pragma once
#include <srrg_property/property.h>

#include "iteration_algorithm_base.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  class IterationAlgorithmLM : public IterationAlgorithmBase {
  public:
    PARAM(PropertyFloat,
          user_lambda_init,
          "initial lm lambda, if 0 is computed by system [default: 0]",
          0,
          0);
    PARAM(PropertyFloat, step_high, "upper clamp for lambda if things go well", 2. / 3., 0);
    PARAM(PropertyFloat, step_low, "lower clamp for lambda if things go well ", 1. / 3., 0);
    PARAM(PropertyInt, lm_iterations_max, "max lm iterations [default: 10]", 10, 0);
    PARAM(PropertyBool,
          variable_damping,
          "set to true uses lambda*diag(H), otherwise uses lambda*I [default: true]",
          true,
          0);
    PARAM(PropertyFloat,
          tau,
          "scale factor for the lambda computed by the system, do not influence the one provided "
          "by the user [default: 1e-5]",
          1e-5,
          0);

    virtual void setSolver(SolverBase* _solver) override;
    virtual bool oneRound() override;

  protected:
    float computeScale();
    float computeLambdaInit();
    void updateDiagonal();
    void cropScale(float& value_);

    //@brief initial diagonal
    std::vector<float> _diagonal;
    //@brief current diagonal considering the actual lambda
    std::vector<float> _current_diagonal;
    // @brief perturbation vector
    std::vector<float> _dx;
    // @brief RHS vector
    std::vector<float> _b;
    // @brief read the name
    float _lambda = 0.;
    // @brief lambda factor when the chi increase after an lm iteration
    float _ni = 2.;
  };
} // namespace srrg2_solver
