#pragma once
#include <srrg_config/configurable.h>

#include "solver_stats.h"

namespace srrg2_solver {

  using namespace srrg2_core;

  class SolverBase;

  /*! @brief Interface for an iteration algorithm, you just need to override the
    oneRound() method which specify how to perform an iteration. The rest of the methods
    just call the corresponding functions in SolverBase */
  struct IterationAlgorithmBase : public Configurable {
    friend class SolverBase;
    /*! Perform a single iteration of the optimization algorithm */
    virtual bool oneRound() = 0;

  protected:
    /*! See SolverBase */
    bool updateChi(IterationStats& istats);
    /*! See SolverBase */
    bool buildQuadraticForm(IterationStats& istats);
    /*! See SolverBase */
    bool solveQuadraticForm(IterationStats& istats);
    /*! See SolverBase */
    void applyPerturbation(IterationStats& istats);
    /*! See SolverBase */
    void getDiagonal(std::vector<float>& diagonal) const;
    /*! See SolverBase */
    void setDiagonal(const std::vector<float>& diagonal);
    /*! See SolverBase */
    void getRHS(std::vector<float>& b) const;
    /*! See SolverBase */
    void getPerturbation(std::vector<float>& dx) const;
    /*! See SolverBase */
    void push();
    /*! See SolverBase */
    void pop();
    /*! See SolverBase */
    void discardTop();
    /*! See SolverBase */
    const int& currentIteration() const;
    /*! See SolverBase */
    IterationStatsVector& iterationStats();
    /*! Setter for the solver pointer at which this optimization algorithm is connected
      @param[in] solver_ pointer to SolverBase
    */
    virtual void setSolver(SolverBase* solver_);

    SolverBase* _solver = 0; /*!< Pointer to the solver that used this algorithm */
  };
  /*! Shared pointer to IterationAlgorithmBase */
  using IterationAlgorithmBasePtr = std::shared_ptr<IterationAlgorithmBase>;
} // namespace srrg2_solver
