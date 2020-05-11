#pragma once
#include "sparse_block_matrix.h"

namespace srrg2_solver {
  /*! @brief Block cholesky matrix,
   you instantiate it from a SparseBlockMatrix and on construction it computes
   the Cholesky Factorization of the matrix. Further this class can be used to solve a linear system
   through choleskySolve()
  */
  class SparseBlockCholesky : public SparseBlockMatrix {
  public:
    /*! Construct the Cholesky factor from block matrix (compute the factorization)
      @param[in] other block matrix to be factorized
    */
    SparseBlockCholesky(const SparseBlockMatrix& other = SparseBlockMatrix());

    /*! Allocates the cholesky block structure
     @return false on failure
    */
    bool choleskyAllocate();

    /*! Updates the coefficients
      @return false on failure
    */
    bool choleskyUpdate();

    /*! Solve a linear system with target vector x which is the overwritten with then solution
      @param[in,out] x target vector
    @return false on failure
    */
    bool choleskySolve(SparseBlockMatrix& x) const;

    /*! Solve a linear system with target block matrix x which is then overwritten with the
    solution. The system is solved by columns
      @param[in,out] x target block matrix
    @return false on failure
    */
    bool blockCholeskySolve(IntBlockMap& x) const;

  protected:
    bool structureScalarColProd(int col_idx_i, int col_idx_j) const;
    int scalarColProd(MatrixBlockBase* dest, int col_idx_i, int col_idx_j) const;
    enum ErrorType { ErrorWarning, ErrorAbort, ErrorFatal };
    void cholError(ErrorType type,
                   const char* message,
                   int row_idx,
                   int col_idx,
                   const MatrixBlockBase* block = 0) const;
    bool forwardSubstitute(float* x) const;
    bool backSubstitute(float* x) const;

    /*! Block version of forward substitutions
     @return false on failure
     */
    bool blockForwardSubstitution(IntBlockMap& x) const;
    /*! Block version of backward substitutions
     @return false on failure
     */
    bool blockBackwardSubstitution(IntBlockMap& x) const;

    std::vector<MatrixBlockBasePtr> _inv_Ljj; /*!< cache inverse diagonal blocks */
  };
} // namespace srrg2_solver
