#pragma once
#include <srrg_property/property.h>

#include "iteration_algorithm_base.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  /*! @brief Levenberg–Marquardt optimization algorithm */
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
    /*! Compute the squared error variation in the linear model. The ratio between the variation
      of the non-linear model and this scale determine if the lm iteration was successful or not
      @return linear error model variation
     */
    float computeScale();
    /*! Compute initial value for the dumping factor
      @return Initial damping
    */
    float computeLambdaInit();
    /*! Update diagonal with new damping */
    void updateDiagonal();
    /*! Limit the reduction factor for the dumping after a succesful iteration */
    void cropScale(float& value_);

    std::vector<float> _diagonal; /*!< Initial diagonal of the H matrix*/

    std::vector<float> _current_diagonal; /*!< Current diagonal
                                            considering the actual lambda */

    std::vector<float> _dx; /*!< Perturbation vector */

    std::vector<float> _b; /*!< Target vector */

    float _lambda = 0.; /*!< Current dumping factor */

    float _ni = 2.; /*!< When the chi increase after an lm iteration
                      the damping is multiplied by _ni.
                      Further for each failure _ni is doubled */
  };

} // namespace srrg2_solver
