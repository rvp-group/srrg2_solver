#include "sparse_block_linear_solver_cholesky.h"

namespace srrg2_solver {

  SparseBlockLinearSolver::Status SparseBlockLinearSolverCholesky::updateStructure() {
    assert(_A && _b && " A matrix null");
    _L = SparseBlockCholesky(*_A);
    _x = SparseBlockMatrix(_b->blockRowDims(), _b->blockColDims());
    if (_L.choleskyAllocate()) {
      return SparseBlockLinearSolver::StructureGood;
    }
    return SparseBlockLinearSolver::StructureBad;
    _structure_changed = false;
  }

  SparseBlockLinearSolver::Status SparseBlockLinearSolverCholesky::updateCoefficients() {
    _L.setZero();
    _A->copyValues(_L);
    if (_L.choleskyUpdate()) {
      return SparseBlockLinearSolver::CoefficientsGood;
    }
    return SparseBlockLinearSolver::CoefficientsBad;
    _coefficients_changed = false;
  }

  SparseBlockLinearSolver::Status SparseBlockLinearSolverCholesky::updateSolution() {
    _b->copyValues(_x);
    if (_L.choleskySolve(_x)) {
      return SparseBlockLinearSolver::SolutionGood;
    }
    return SparseBlockLinearSolver::SolutionBad;
  }

  bool
  SparseBlockLinearSolverCholesky::computeBlockInverse(SparseBlockMatrix& inverse_blocks,
                                                       const std::vector<IntPair>& blocks_layout) {
    std::set<int> interest_row_indicies;

    // fill the matrix with block layout
    for (const IntPair& row_column_idx : blocks_layout) {
      MatrixBlockBase* block =
        inverse_blocks.blockAt(row_column_idx.second, row_column_idx.second, true);
      block->setIdentity();
      if (row_column_idx.first != row_column_idx.second) {
        block = inverse_blocks.blockAt(row_column_idx.first, row_column_idx.second, true);
        block->setZero();
      }
    }
    // solver linear system
    for (SparseBlockMatrix::IntBlockMap& col : inverse_blocks._cols) {
      if (!col.empty()) {
        for (const auto& row_element : col) {
          interest_row_indicies.insert(row_element.first);
        }

        if (!_L.blockCholeskySolve(col)) {
          return false;
        }
        // remove duplicated blocks
        for (auto it = col.begin(); it != col.end(); ++it) {
          const auto& idx_it = interest_row_indicies.find(it->first);
          if (idx_it == interest_row_indicies.end()) {
            col.erase(it);
          }
        }
        interest_row_indicies.clear();
      }
    }

    return true;
  }

} // namespace srrg2_solver
