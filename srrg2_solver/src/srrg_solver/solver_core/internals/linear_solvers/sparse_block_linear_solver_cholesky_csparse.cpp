#include <suitesparse/amd.h>
#include <suitesparse/cholmod.h>
#include <suitesparse/colamd.h>
#include <suitesparse/cs.h>

#include "sparse_block_linear_solver_cholesky_csparse.h"

namespace srrg2_solver {

  void SparseBlockLinearSolverCholeskyCSparse ::computeOrderingHint(
    std::vector<int>& ordering,
    const std::vector<IntPair>& layout) const {
    const int dim    = ordering.size();
    const size_t nnz = layout.size();
    cs* cs_matrix    = cs_spalloc(dim, dim, nnz, 1, 0);
    int* col_ptr     = cs_matrix->p;
    int* row_ptr     = cs_matrix->i;
    double* val_ptr  = cs_matrix->x;
    int previous_col = -1;
    for (size_t i = 0; i < nnz; ++i) {
      int r = layout[i].first;
      int c = layout[i].second;
      if (c != previous_col) {
        *col_ptr = i;
        ++col_ptr;
        previous_col = c;
      }
      *row_ptr = r;
      *val_ptr = 1.0;
      ++row_ptr;
      ++val_ptr;
    }
    *col_ptr  = nnz;
    int* perm = cs_amd(1, cs_matrix);
    cs_spfree(cs_matrix);
    if (!perm) {
      ordering.clear();
      return;
    }
    ordering.resize(dim);
    for (int i = 0; i < dim; ++i) {
      ordering[i] = perm[i];
    }
    cs_free(perm);
  }

} // namespace srrg2_solver
