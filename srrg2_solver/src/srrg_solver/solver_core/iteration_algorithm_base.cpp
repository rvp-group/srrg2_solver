#include "iteration_algorithm_base.h"
#include "solver_base.h"

namespace srrg2_solver {

  bool IterationAlgorithmBase::updateChi(IterationStats& istats) {
    return _solver->updateChi(istats);
  }

  bool IterationAlgorithmBase::buildQuadraticForm(IterationStats& istats) {
    return _solver->buildQuadraticForm(istats);
  }

  bool IterationAlgorithmBase::solveQuadraticForm(IterationStats& istats) {
    return _solver->solveQuadraticForm(istats);
  }

  void IterationAlgorithmBase::applyPerturbation(IterationStats& istats) {
    _solver->applyPerturbation(istats);
  }

  void IterationAlgorithmBase::getDiagonal(std::vector<float>& diagonal) const {
    _solver->getDiagonal(diagonal);
  }

  void IterationAlgorithmBase::setDiagonal(const std::vector<float>& diagonal) {
    _solver->setDiagonal(diagonal);
  }

  void IterationAlgorithmBase::getRHS(std::vector<float>& b) const {
    _solver->getRHS(b);
  }

  void IterationAlgorithmBase::getPerturbation(std::vector<float>& dx) const {
    _solver->getPerturbation(dx);
  }

  void IterationAlgorithmBase::push() {
    _solver->push();
  }

  void IterationAlgorithmBase::pop() {
    _solver->pop();
  }

  void IterationAlgorithmBase::discardTop() {
    _solver->discardTop();
  }

  const int& IterationAlgorithmBase::currentIteration() const {
    return _solver->currentIteration();
  }

  IterationStatsVector& IterationAlgorithmBase::iterationStats() {
    return _solver->_iteration_stats;
  }

  void IterationAlgorithmBase::setSolver(SolverBase* solver_) {
    _solver = solver_;
  }

} // namespace srrg2_solver
