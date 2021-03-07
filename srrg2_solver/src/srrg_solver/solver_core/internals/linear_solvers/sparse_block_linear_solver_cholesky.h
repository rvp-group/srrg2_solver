#pragma once
#include "sparse_block_linear_solver.h"
#include "srrg_solver/solver_core/internals/sparse_block_matrix/sparse_block_cholesky.h"

namespace srrg2_solver {

  // naive block solver that uses the cholesky block decomposition
  // no ordering is applied
  class SparseBlockLinearSolverCholesky : public SparseBlockLinearSolver {
  public:
    // compute inverse of specific block of A
    bool computeBlockInverse(SparseBlockMatrix& inverse_blocks,
                             const std::vector<IntPair>& blocks_layout) override;

  protected:
    // computes the internal structure, given the structure of A
    virtual Status updateStructure();

    // copies the coefficients, and computes the numerical structure for the solver
    virtual Status updateCoefficients();

    // solves the linear system based on the updated coefficients
    virtual Status updateSolution();


    SparseBlockCholesky _L;
  };
} // namespace srrg2_solver
