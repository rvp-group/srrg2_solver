#pragma once
#include <srrg_config/configurable.h>

#include "solver_stats.h"

namespace srrg2_solver {

  using namespace srrg2_core;

  class SolverBase;

  // pure virtual class for an iteration algorithm
  struct IterationAlgorithmBase : public Configurable {
    friend class SolverBase;

    virtual bool oneRound() = 0; // implement this to do one iteration

  protected:
    bool updateChi(IterationStats& istats);

    bool buildQuadraticForm(IterationStats& istats);

    bool solveQuadraticForm(IterationStats& istats);

    void applyPerturbation(IterationStats& istats);

    void getDiagonal(std::vector<float>& diagonal) const;

    void setDiagonal(const std::vector<float>& diagonal);

    void getRHS(std::vector<float>& b) const;

    void getPerturbation(std::vector<float>& dx) const;

    void push();

    void pop();

    void discardTop();

    const int& currentIteration() const;

    IterationStatsVector& iterationStats();

    virtual void setSolver(SolverBase* solver_);

    SolverBase* _solver = 0;
  };

  using IterationAlgorithmBasePtr = std::shared_ptr<IterationAlgorithmBase>;
} // namespace srrg2_solver
