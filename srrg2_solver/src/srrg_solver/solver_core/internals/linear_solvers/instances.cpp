#include "instances.h"
#include "sparse_block_linear_solver_cholesky.h"
#include "sparse_block_linear_solver_cholesky_cholmod.h"
#include "sparse_block_linear_solver_cholesky_csparse.h"
#include "sparse_block_linear_solver_cholesky_emd.h"
#include "sparse_block_linear_solver_cholmod_full.h"

namespace srrg2_solver {

  void linear_solver_registerTypes() {
    BOSS_REGISTER_CLASS(SparseBlockLinearSolverCholeskyCSparse);
    BOSS_REGISTER_CLASS(SparseBlockLinearSolverCholeskyCholmod);
    BOSS_REGISTER_CLASS(SparseBlockLinearSolverCholmodFull);
    BOSS_REGISTER_CLASS(SparseBlockLinearSolverCholesky);
    BOSS_REGISTER_CLASS(SparseBlockLinearSolverCholeskyEMD);
  }
} // namespace srrg2_solver
