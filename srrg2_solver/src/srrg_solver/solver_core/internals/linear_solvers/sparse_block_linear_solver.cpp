#include "sparse_block_linear_solver.h"
#include <iostream>

namespace srrg2_solver {
  using namespace srrg2_core;
  using namespace std;

  const char* SparseBlockLinearSolver::_string_status[] = {"Error",
                                                           "SolutionGood",
                                                           "SolutionBad",
                                                           "StructureBad",
                                                           "StructureGood",
                                                           "CoefficientsBad",
                                                           "CoefficientsGood"};

  SparseBlockLinearSolver::SparseBlockLinearSolver() {
    _structure_changed    = true;
    _coefficients_changed = true;
  }

  void SparseBlockLinearSolver::setX(const SparseBlockMatrix& x_) {
    if (!_b) {
      throw std::runtime_error("SparseBlockLinearSolver::setX| no b vector set");
    }
    _x = SparseBlockMatrix(_b->blockRowDims(), _b->blockColDims());
    x_.copyValues(_x);
  }

  void SparseBlockLinearSolver::compute() {
    if (_structure_changed) {
      _status = updateStructure();
    }

    if (_status == Error || _status == StructureBad) {
      return;
    }

    if (_coefficients_changed) {
      _status = updateCoefficients();
    }

    if (_status != CoefficientsGood) {
      return;
    }
    _status = updateSolution();
  }

  void
  SparseBlockLinearSolver::computeOrderingHint(std::vector<int>& ordering,
                                               const std::vector<IntPair>& block_layout) const {
    for (size_t i = 0; i < ordering.size(); ++i) {
      ordering[i] = i;
    }
  }

} // namespace srrg2_solver
