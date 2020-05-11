#pragma once
#include <srrg_property/property.h>

#include "iteration_algorithm_base.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  /*! @brief Gauss-Netwon optimization algorithm. In our implementation you can set
    a fixed damping factor to enforce full rank H matrix */
  class IterationAlgorithmGN : public IterationAlgorithmBase {
  public:
    PARAM(PropertyFloat,
          damping,
          "damping factor, the higher the closer to gradient descend. Default:0",
          0,
          0);

    virtual void setSolver(SolverBase* _solver) override;
    virtual bool oneRound() override;

  protected:
    std::vector<float> _diagonal; /*!< Store the diagonal of the H matrix */
  };
} // namespace srrg2_solver
