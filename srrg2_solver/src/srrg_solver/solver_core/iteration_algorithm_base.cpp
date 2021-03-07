#include "iteration_algorithm_base.h"
#include "solver.h"

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

  void IterationAlgorithmBase::setPerturbation(const std::vector<float>& dx) const {
    _solver->setPerturbation(dx);
  }

  float IterationAlgorithmBase::HessianSymmetricProduct(const std::vector<float>& v) const {
    Solver* solver = static_cast<Solver*>(_solver);
    if (!solver) {
      std::cerr << "IterationAlgorithmBase::HessianSymmetricProduct| the associated solver "
                   "is not of class Solver"
                << std::endl;
      return 0.f;
    }
    SparseBlockMatrix vec(solver->_b.blockRowDims(), solver->_b.blockColDims());
    int idx = 0;
    for (int rb = 0; rb < vec.blockRows(); ++rb) {
      MatrixBlockBase* row_block = vec.blockAt(rb, 0, true);
      int block_dim              = vec.blockDims(rb, 0).first;
      for (int r = 0; r < block_dim; ++r, ++idx) {
        row_block->at(r, 0) = v.at(idx);
      }
    }
    return solver->_H.symmetricProduct(vec);
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
