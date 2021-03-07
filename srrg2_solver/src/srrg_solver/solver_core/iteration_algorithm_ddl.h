#pragma once
#include "iteration_algorithm_dl.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  /* @brief Double DogLeg optimization algorithm with ellipsoidal trust region
   * see IterationAlgorithmDL for the implementation of the inner loop and the computation of the
   * steps */
  class IterationAlgorithmDDL : public IterationAlgorithmDL {
  protected:
    /*! Double dogleg step computation */
    void _computeStep(float& linear_decrease, const float& current_chi) final;
  };
} // namespace srrg2_solver
