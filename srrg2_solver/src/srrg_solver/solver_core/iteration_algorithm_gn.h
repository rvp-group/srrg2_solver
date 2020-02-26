#pragma once
#include <srrg_property/property.h>

#include "iteration_algorithm_base.h"

namespace srrg2_solver {
  using namespace srrg2_core;

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
    std::vector<float> _diagonal;
  };
} // namespace srrg2_solver
