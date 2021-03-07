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

  void SparseBlockLinearSolverCholmodFull::_cacheInverseDiagonal() {
    if (!_L) {
      return;
    }
    cholmod_change_factor(CHOLMOD_REAL, 1, 0, 1, 1, _L, &_cholmodCommon);
    assert(_L->is_ll && !_L->is_super && _L->is_monotonic &&
           "SparseBlockLinearSolverCholmodFull::_cacheInverseDiagonal()|Cholesky factor has wrong "
           "format");

    _inverse_diagonal.clear();
    int n                           = _A->cols();
    double* cholesky_factor_entries = (double*) _L->x;
    int* column_pointers            = (int*) _L->p;
    assert(
      cholesky_factor_entries != nullptr &&
      "SparseBlockLinearSolverCholmodFull::_cacheInverseDiagonal()| cholmod_factor coefficients "
      "pointer is void");

    assert(column_pointers != nullptr &&
           "SparseBlockLinearSolverCholmodFull::_cacheInverseDiagonal()| cholmod_factor column "
           "pointers empty");
    _inverse_diagonal.resize(n);
    double inv_diag = 0;
    for (int c = 0; c < n; ++c) {
      const int& diagonal_index = column_pointers[c];
      inv_diag                  = 1.0 / cholesky_factor_entries[diagonal_index];
      assert(inv_diag == inv_diag &&
             "SparseBlockLinearSolverCholmodFull::_cacheInverseDiagonal| nan entry in diagonal");
      _inverse_diagonal[c] = inv_diag;
    }
  }

  SparseBlockLinearSolver::Status SparseBlockLinearSolverCholmodFull::updateStructure() {
    // tg check if the system matrix has a symmetric layout
    if (!_A->isLayoutSymmetric()) {
      return SparseBlockLinearSolver::StructureBad;
    }
    // tg free the relevant cholmod matricies
    if (_B != nullptr) {
      cholmod_free_sparse(&_B, &_cholmodCommon);
    }
    if (_c != nullptr) {
      cholmod_free_dense(&_c, &_cholmodCommon);
    }
    if (_L != nullptr) {
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
    // _B->p       = new int[rows + 1];
    // _B->i       = new int[_B->nzmax];
    // _B->x       = new double[_B->nzmax];
    _A->toCCS((int*) _B->i, (int*) _B->p, (double*) _B->x);
    // apply null ordering (block ordering is performed by the Solver at the level of
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
    _elements_cache.clear();
    return SparseBlockLinearSolver::CoefficientsGood;
  }

  SparseBlockLinearSolver::Status SparseBlockLinearSolverCholmodFull::updateSolution() {
    // tg delete previous solution
    if (_solution != nullptr) {
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

  double SparseBlockLinearSolverCholmodFull::_computeMatrixEntry(const int& r, const int& c) {
    assert(r <= c && "SparseBlockLinearSolverCholmodFull::_computeMatrixEntry()| r > c");
    int idx                                = r * _A->rows() + c;
    MatrixElementsCache::const_iterator it = _elements_cache.find(idx);
    if (it != _elements_cache.end()) {
      return it->second;
    }
    // compute the summation over column r
    double sum_row                  = 0.;
    double* cholesky_factor_entries = (double*) _L->x;
    int* column_pointers            = (int*) _L->p;
    int* row_indices                = (int*) _L->i;
    const int& sc                   = column_pointers[r];
    const int& ec                   = column_pointers[r + 1];
    for (int j = sc + 1; j < ec; ++j) { // sum over row r while skipping the element on the diagonal
      const int& rr = row_indices[j];
      double val    = rr < c ? _computeMatrixEntry(rr, c) : _computeMatrixEntry(c, rr);
      sum_row += val * cholesky_factor_entries[j];
    }

    double result;
    const double& inv_Lrr = _inverse_diagonal[r];
    assert(
      inv_Lrr == inv_Lrr &&
      "SparseBlockLinearSolverCholmodFull::_computeMatrixEntry| inverse diagonal element is nan");
    if (r == c) {
      result = inv_Lrr * (inv_Lrr - sum_row);
    } else {
      result = -sum_row * inv_Lrr;
    }
    _elements_cache[idx] = result;
    return result;
  }

  bool SparseBlockLinearSolverCholmodFull::computeBlockInverse(
    SparseBlockMatrix& inverse_blocks,
    const std::vector<IntPair>& blocks_layout) {
    _cacheInverseDiagonal();
    if (_inverse_diagonal.empty()) {
      std::cerr << "SparseBlockLinearSolverCholmodFull::computeBlockInverse| cache for inverse "
                   "diagonal elements of cholesky factor is empty"
                << std::endl;
      return false;
    }
    // tg set the matrix elements to be computed starting from the
    // block pattern
    std::vector<IntPair> elements_to_compute;
    // tg for each block extract starting absolute indices and block dimensions
    for (const IntPair& block_indices : blocks_layout) {
      const IntPair block_offsets    = _A->blockOffsets(block_indices.first, block_indices.second);
      const IntPair block_dimensions = _A->blockDims(block_indices.first, block_indices.second);

      for (int r = 0; r < block_dimensions.first; ++r) {
        for (int c = 0; c < block_dimensions.second; ++c) {
          // tg compute absolute element indices
          int row_idx_block_matrix = r + block_offsets.first;
          int col_idx_block_matrix = c + block_offsets.second;

          // fix upper triangular structure
          if (row_idx_block_matrix > col_idx_block_matrix) {
            std::swap(row_idx_block_matrix, col_idx_block_matrix);
          }
          elements_to_compute.push_back(IntPair(row_idx_block_matrix, col_idx_block_matrix));
        }
      }
    }

    // tg sort the elements so that elements at last columns
    // are processed before
    std::sort(elements_to_compute.begin(),
              elements_to_compute.end(),
              [](const IntPair& a, const IntPair& b) -> bool {
                return a.second > b.second || (a.second == b.second && a.first > b.first);
              });
    // tg compute the required matrix elements
    for (const IntPair& elem : elements_to_compute) {
      _computeMatrixEntry(elem.first, elem.second);
    }

    // tg fill the blocks with the values
    for (const IntPair& block_idx : blocks_layout) {
      MatrixBlockBase* block      = inverse_blocks.blockAt(block_idx.first, block_idx.second, true);
      const IntPair block_offsets = _A->blockOffsets(block_idx.first, block_idx.second);
      const IntPair block_dimensions = _A->blockDims(block_idx.first, block_idx.second);
      for (int r = 0; r < block_dimensions.first; ++r) {
        for (int c = 0; c < block_dimensions.second; ++c) {
          int row_idx_block_matrix = r + block_offsets.first;
          int col_idx_block_matrix = c + block_offsets.second;
          if (row_idx_block_matrix > col_idx_block_matrix) {
            std::swap(row_idx_block_matrix, col_idx_block_matrix);
          }

          int idx = row_idx_block_matrix * _A->rows() + col_idx_block_matrix;
          MatrixElementsCache::const_iterator it = _elements_cache.find(idx);
          assert(it != _elements_cache.end() &&
                 "SparseBlockLinearSolverCholmodFull::computeBlockInverse| something broken in the "
                 "cached elements");

          block->at(r, c) = static_cast<float>(it->second);
          if (block_idx.first == block_idx.second) {
            block->at(c, r) = static_cast<float>(it->second);
          }
        }
      }
    }
    return true;
  }
} // namespace srrg2_solver
