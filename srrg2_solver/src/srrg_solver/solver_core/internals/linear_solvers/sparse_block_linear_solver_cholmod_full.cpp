#include "sparse_block_linear_solver_cholmod_full.h"
#include <numeric>

namespace srrg2_solver {

  SparseBlockLinearSolverCholmodFull::SparseBlockLinearSolverCholmodFull() {
    cholmod_start(&_cholmodCommon);
    // tg set up cholmod parameters
    _cholmodCommon.nmethods           = 0;
    _cholmodCommon.method[0].ordering = CHOLMOD_GIVEN;
    _cholmodCommon.supernodal         = CHOLMOD_AUTO;
    _cholmodCommon.final_ll           = 1;
  }

  SparseBlockLinearSolverCholmodFull::~SparseBlockLinearSolverCholmodFull() {
    // tg free workspace and terminate cholmod
    cholmod_free_sparse(&_B, &_cholmodCommon);
    cholmod_free_dense(&_c, &_cholmodCommon);
    cholmod_free_dense(&_solution, &_cholmodCommon);
    cholmod_free_factor(&_L, &_cholmodCommon);
    cholmod_finish(&_cholmodCommon);
  }

  SparseBlockLinearSolver::Status SparseBlockLinearSolverCholmodFull::updateStructure() {
    // tg check if the system matrix has a symmetric layout
    if (!_A->isLayoutSymmetric()) {
      return SparseBlockLinearSolver::StructureBad;
    }
    // tg free the relevant cholmod matricies
    if (_B != NULL) {
      cholmod_free_sparse(&_B, &_cholmodCommon);
    }
    if (_c != NULL) {
      cholmod_free_dense(&_c, &_cholmodCommon);
    }
    if (_L != NULL) {
      cholmod_free_factor(&_L, &_cholmodCommon);
    }
    // tg allocate cholmod dense for target vector _c
    const size_t num_rows = static_cast<size_t>(_b->rows());
    const size_t num_cols = static_cast<size_t>(_b->cols());

    assert(num_cols == 1 &&
           "SparseBlockLinearSolverCholmodFull::updateStructure| b vector should have one column");
    _c = cholmod_zeros(num_rows, num_cols, CHOLMOD_REAL, &_cholmodCommon);

    // tg allocate cholmod sparse for the system matrix
    size_t rows = _A->rows();
    size_t cols = _A->cols();
    size_t nnz  = _A->numNonZeros();
    _B          = cholmod_allocate_sparse(rows, cols, nnz, 1, 1, 1, CHOLMOD_REAL, &_cholmodCommon);
    _B->p       = new int[rows + 1];
    _B->i       = new int[_B->nzmax];
    _B->x       = new double[_B->nzmax];
    _A->toCCS((int*) _B->i, (int*) _B->p, (double*) _B->x);
    // apply null ordering (block ordering is performed by the SparseSolver at the level of
    // variables through computeOrderingHint, see below)
    int* null_permutation = new int[rows];
    std::unique_ptr<int[]> null_permutation_ptr(null_permutation);

    std::iota(null_permutation, null_permutation + rows, 0);
    _L = cholmod_analyze_p(_B, null_permutation, NULL, 0, &_cholmodCommon);

    // tg allocate solution in SparseBlockMatrix format
    _x = SparseBlockMatrix(_b->blockRowDims(), _b->blockColDims());

    return SparseBlockLinearSolver::StructureGood;
  }

  SparseBlockLinearSolver::Status SparseBlockLinearSolverCholmodFull::updateCoefficients() {
    // tg copy system matrix in cholmod format
    _A->fillCCS((double*) _B->x);
    cholmod_factorize(_B, _L, &_cholmodCommon);

    // tg check if matrix is positive definite
    if (_cholmodCommon.status == CHOLMOD_NOT_POSDEF) {
      return SparseBlockLinearSolver::CoefficientsBad;
    }
    return SparseBlockLinearSolver::CoefficientsGood;
  }

  SparseBlockLinearSolver::Status SparseBlockLinearSolverCholmodFull::updateSolution() {
    // tg delete previous solution
    if (_solution != NULL) {
      cholmod_free_dense(&_solution, &_cholmodCommon);
    }
    // tg fill target vector and solve the linear system
    _b->fillCCS((double*) _c->x, false);
    _solution = cholmod_solve(CHOLMOD_A, _L, _c, &_cholmodCommon);

    // tg need to cast back to float to make it compatible with SparseBlockMatrix
    // check if solution containts nan of inf
    const double* data = (double*) (_solution->x);
    for (int row_block_idx = 0; row_block_idx < _x.blockRows(); ++row_block_idx) {
      MatrixBlockBase* block       = _x.blockAt(row_block_idx, 0, true);
      auto offsets                 = _x.blockOffsets(row_block_idx, 0);
      int row_offset               = offsets.first;
      float* storage               = block->storage();
      const double* data_row_block = data + row_offset;
      for (int r = 0; r < block->rows(); ++r, ++storage, ++data_row_block) {
        if (!std::isfinite(*data_row_block)) {
          return SparseBlockLinearSolver::SolutionBad;
        }
        *storage = static_cast<float>(*data_row_block);
      }
    }

    return SparseBlockLinearSolver::SolutionGood;
  }

  void SparseBlockLinearSolverCholmodFull::computeOrderingHint(
    std::vector<int>& ordering,
    const std::vector<IntPair>& layout) const {
    const int dim    = ordering.size();
    const size_t nnz = layout.size();

    // tg convert the block structure to cholmod triplets
    cholmod_triplet triplet;

    triplet.itype = CHOLMOD_INT;
    triplet.xtype = CHOLMOD_REAL;
    triplet.dtype = CHOLMOD_DOUBLE;

    triplet.nrow  = static_cast<size_t>(dim);
    triplet.ncol  = static_cast<size_t>(dim);
    triplet.nzmax = static_cast<size_t>(nnz);
    triplet.nnz   = static_cast<size_t>(nnz);
    triplet.stype = 1;

    int* i_temp    = new int[nnz];
    int* j_temp    = new int[nnz];
    double* x_temp = new double[nnz];
    std::unique_ptr<int[]> i_ptr(i_temp);
    std::unique_ptr<int[]> j_ptr(j_temp);
    std::unique_ptr<double[]> x_ptr(x_temp);

    triplet.i = i_temp;
    triplet.j = j_temp;
    triplet.x = x_temp;

    for (size_t i = 0; i < nnz; ++i) {
      int r     = layout[i].first;
      int c     = layout[i].second;
      i_temp[i] = r;
      j_temp[i] = c;
      x_temp[i] = 1;
    }
    // tg needed to use AMD ordering and free sparse matrix
    cholmod_common* cc_no_const = const_cast<cholmod_common*>(&_cholmodCommon);
    // tg convert triplets to sparse matrix
    cholmod_sparse* ch_sparse = cholmod_triplet_to_sparse(&triplet, triplet.nnz, cc_no_const);
    ch_sparse->xtype          = CHOLMOD_PATTERN;

    cholmod_amd(ch_sparse, NULL, 0, ordering.data(), cc_no_const);
    cholmod_free_sparse(&ch_sparse, cc_no_const);
  }
} // namespace srrg2_solver
