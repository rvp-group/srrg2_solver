#include "iteration_algorithm_gn.h"
namespace srrg2_solver {
  using namespace srrg2_core;

  void IterationAlgorithmGN::setSolver(SolverBase* solver_) {
    IterationAlgorithmBase::setSolver(solver_);
    if (_solver) {
      getDiagonal(_diagonal);
    }
  }

  bool IterationAlgorithmGN::oneRound() {
    //std::cerr << "one round" << std::endl;
    IterationStats istats;
    istats.reset();
    istats.iteration = currentIteration();
    // bool success=true;
    if (!buildQuadraticForm(istats)) {
      iterationStats().push_back(istats);
      //std::cerr << "QF fail" << std::endl;
      return false;
    }

    if (param_damping.value() > 0) {
      istats.lambda = param_damping.value();
      getDiagonal(_diagonal);
      for (size_t i = 0; i < _diagonal.size(); ++i) {
        _diagonal[i] += param_damping.value();
      }
      setDiagonal(_diagonal);
    }
    if (!solveQuadraticForm(istats)) {
      //std::cerr << "solve fail" << std::endl;
      iterationStats().push_back(istats);
      return false;
    }
    applyPerturbation(istats);
    iterationStats().push_back(istats);
    return true;
  }
} // namespace srrg2_solver
