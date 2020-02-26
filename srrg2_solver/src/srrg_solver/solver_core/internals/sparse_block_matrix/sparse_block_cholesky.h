#pragma once
#include "sparse_block_matrix.h"

namespace srrg2_solver {
  // block cholesky decomposition
  // you instantiate it from a block matrix
  // usage:
  //  SparseBlockCholesky L(A); // L will be the cholesky matrix
  //
  // each time A changes the structure
  //  L.choleskyAllocate();
  //
  // each time A changes the values
  //  L.choleskyUpdate()
  //
  // if you want to solve the linear system
  //  A x = b
  //
  //  SparseBlockCholesky L(A);
  //  L.choleskyAllocate();
  //  L.choleskyUpdate()
  //  L.choleskySolve(x);


  class SparseBlockCholesky: public SparseBlockMatrix {
  public:
    SparseBlockCholesky(const SparseBlockMatrix& other = SparseBlockMatrix());

    // allocates the cholesky structure
    // @ returns false on failure
    bool choleskyAllocate();

    // updates the cholesky values
    // @returns false on failure
    bool choleskyUpdate();

    // solve the syste,
    // @returns false on failure
    bool choleskySolve(SparseBlockMatrix& x) const;

    // solve the linear system for a block column
    // @return false on failure
    bool blockCholeskySolve(IntBlockMap& x) const;

  protected:
    // cholesky stuff
    bool structureScalarColProd(int col_idx_i, int col_idx_j) const;
    // returns number of operations
    int scalarColProd(MatrixBlockBase* dest, int col_idx_i, int col_idx_j) const;
    enum ErrorType { ErrorWarning, ErrorAbort, ErrorFatal };
    void cholError(ErrorType type,
                   const char* message,
                   int row_idx,
                   int col_idx,
                   const MatrixBlockBase* block = 0) const;
    bool forwardSubstitute(float* x) const;
    bool backSubstitute(float* x) const;

    // Block version of back/forward substitutions
    bool blockForwardSubstitution(IntBlockMap& x) const;
    bool blockBackwardSubstitution(IntBlockMap& x) const;
    // cache inverse diagonal blocks
    std::vector<MatrixBlockBasePtr> _inv_Ljj;

  };
} // namespace srrg2_solver
