#pragma once
#include "matrix_block.h"

namespace srrg2_solver {
  // @brief triplet structure complaint with cholmod and csparse
  struct Triplets {
    Triplets();
    // @brief number of non zero entries
    size_t nnz;
    // @brief number of rows
    size_t rows;
    // @brief number of cols
    size_t cols;
    // @brief row indicies of non zero elements
    std::vector<int> row_indicies;
    // @brief col indicies of non zero elements
    std::vector<int> col_indicies;
    // @brief values
    std::vector<double> data;
  };
  /*! @brief Simplistic block matrix class with column major ordering.
    The blocks are owned by a matrix */
  class SparseBlockMatrix {
    friend class SparseBlockCholesky;
    friend class SparseBlockLinearSolverCholesky;

  public:
    using IntBlockMap = std::map<int, MatrixBlockBasePtr>; /*!< Column represented
                                                             as index-Block unique pointer */
    using IntBlockMapVector = std::vector<IntBlockMap>;    /*!< Columns as vector of IntBlockMap */

    /*! Allocates a matrix with a specified row and column block layout */
    SparseBlockMatrix(const std::vector<int>& row_block_dims = std::vector<int>(),
                      const std::vector<int>& col_block_dims = std::vector<int>());

    /*! Copies the values to dest. requires dest to have the same block structure
      @param[in] dest block matrix
    */
    void copyValues(SparseBlockMatrix& dest) const;

    /*! @return Number of block rows */
    inline int blockRows() const {
      return _row_block_dims.size();
    }

    /*! @return Number of block columns */
    inline int blockCols() const {
      return _col_block_dims.size();
    }

    /*! @return Row block layout (vector of size blockRows,
      the i-th component contains the row dimension of the block at row i */
    inline const std::vector<int>& blockRowDims() const {
      return _row_block_dims;
    }

    /*! @return Row block layout (vector of size blockRows,
      the i-th component contains the row dimension of the block at row i */
    inline const std::vector<int>& blockColDims() const {
      return _col_block_dims;
    }

    /*! @return pair<first=rows, second=cols> containing the dimensions of the block */
    inline IntPair blockDims(int block_row, int block_col) const {
      return IntPair(_row_block_dims[block_row], _col_block_dims[block_col]);
    }

    /*! @return pair<first=rows, second=cols> containing the offset in elements of the
      block selected as argument */
    inline IntPair blockOffsets(int block_row, int block_col) const {
      return IntPair(_row_block_offsets[block_row], _col_block_offsets[block_col]);
    }

    int numNonZeroBlocks() const;

    /*! Accessor for a block
      @param[in] block_row row block index
      @param[in] block_col column block index
      @param[in] create if true, the block is added to the structure if not present
      @return Block pointer
    */
    MatrixBlockBase* blockAt(int block_row, int block_col, bool create = false);

    /*! Accessor for a block (read only)
      @param[in] block_row row block index
      @param[in] block_col column block index
      @return Block pointer
    */
    const MatrixBlockBase* blockAt(int block_row, int block_col) const;

    /*! Delete a block
      @param[in] block_row row block index
      @param[in] block_col column block index
    */
    void blockErase(int block_row, int block_col);

    /*! Release the ownership and delete a block
      @param[in] block_row row block index
      @param[in] block_col column block index
      @return Block pointer
    */
    MatrixBlockBase* blockRelease(int block_row, int block_col);

    /*! @return number of rows */
    inline int rows() const {
      return (!_cols.size()) ? 0 : _row_block_offsets.back();
    }

    /*! @return number of columns */
    inline int cols() const {
      return (!_cols.size()) ? 0 : _col_block_offsets.back();
    }

    /*! Get non zero entries, blocks are assumed to be all full for non symmetric matricies
     while for symmetric structure only the upper triangular elements of the diagonal
     blocks are counted
     @param[in] is_symmetric_
     @return Number of non zero entries
    */
    int numNonZeros(bool is_symmetric_ = true) const;

    /*! Sets to  zero all allocated blocks
      no change in structure */
    void setZero();

    /*! Allocates all possible blocks */
    void allocateFull();

    /*! @return true if the rows and columns block dimensions are the same */
    bool isLayoutSymmetric() const;

    /*! @return true if the structure of self and other is the same (value are not considered) */
    bool isStructureEqual(const SparseBlockMatrix& other) const;

    /*! prints stuff */
    enum PrintMode { PrintFillIn, PrintDim, PrintOffset, PrintValues };
    /*! prints stuff */
    void printLayout(PrintMode mode = PrintMode::PrintValues) const;

    /*! @return Coefficients along the diagonal */
    void getDiagonal(std::vector<float>& diagonal) const;

    /*! Compute product x^T * A * x, where A is this sparse block matrix.
     *  A must be symmetric
     *  @param[in] x sparse block matrix that represent the input vector
     *  @return the product x^T * A * x
     */
    float symmetricProduct(const SparseBlockMatrix& x) const;
    /*! Sets the coefficients along the diagonal
      @param[in] diagonal
    */
    void setDiagonal(const std::vector<float>& diagonal);

    /*! @return Dense vector representing the entries of the matrix */
    void getDenseVector(std::vector<float>& v) const;

    /*! Get the matrix in CCS format (assume upper triangular structure for symmetric matricies)
      @param[out] row_indices_
      @param[out] col_pointers_
      @param[out] values_
      @param[in] is_symmetric_
    */
    void
    toCCS(int* row_indicies_, int* col_pointers_, double* values_, bool is_symmetric_ = true) const;

    /*! Fill the values of a CCS matrix with the value of this block matrix (assume upper triangular
      structure for symmetric matricies)
      @param[out] values_ pointer to values in CSS
      @param[in] is_symmetric_
    */
    void fillCCS(double* values_, bool is_symmetric_ = true) const;

  protected:
    /*! Recompute offsets of block given the current allocation */
    void recomputeOffsets();

    std::vector<int> _row_block_dims;
    std::vector<int> _col_block_dims;
    std::vector<int> _row_block_offsets;
    std::vector<int> _col_block_offsets;
    IntBlockMapVector _cols;
  };

} // namespace srrg2_solver
