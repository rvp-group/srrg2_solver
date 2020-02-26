#include "sparse_block_linear_solver_cholesky_cholmod.h"
#include <suitesparse/cholmod.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  void SparseBlockLinearSolverCholeskyCholmod ::computeOrderingHint(
    std::vector<int>& ordering,
    const std::vector<IntPair>& layout) const {
    cholmod_common cc;
    cholmod_start(&cc);
    const int CHOLMOD_COLAMD_ORDERING = 5;
    cc.nmethods                       = 1;
    cc.method[0].ordering             = CHOLMOD_COLAMD_ORDERING;
    cc.supernodal                     = CHOLMOD_SUPERNODAL;

    const int dim    = ordering.size();
    const size_t nnz = layout.size();

    cholmod_triplet triplet;

    triplet.itype = CHOLMOD_INT;
    triplet.xtype = CHOLMOD_REAL;
    triplet.dtype = CHOLMOD_DOUBLE;

    triplet.nrow  = static_cast<size_t>(dim);
    triplet.ncol  = static_cast<size_t>(dim);
    triplet.nzmax = static_cast<size_t>(nnz);
    triplet.nnz   = static_cast<size_t>(nnz);
    triplet.stype = 1;
    
    int* i_temp = new int[nnz];
    int* j_temp = new int [nnz];
    double* x_temp = new double [nnz];
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
    cholmod_sparse* ch_sparse = cholmod_triplet_to_sparse(&triplet, triplet.nnz, &cc);
    cholmod_factor* factors   = cholmod_analyze(ch_sparse, &cc);
    int* perm                 = (int*) factors->Perm;
    for (int i = 0; i < dim; ++i) {
      ordering[i] = perm[i];
    }

    cholmod_free_factor(&factors, &cc);
    cholmod_free_sparse(&ch_sparse, &cc);
    cholmod_finish(&cc);
  }
} // namespace srrg2_solver
