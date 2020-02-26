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
  //! simplistic block matrix class
  //! column major ordering
  //! the full blocks are owned by a matrix
  class SparseBlockMatrix {
    friend class SparseBlockCholesky;
    friend class SparseBlockLinearSolverCholesky;

  public:
    using IntBlockMap       = std::map<int, MatrixBlockBasePtr>;
    using IntBlockMapVector = std::vector<IntBlockMap>;

    //! allocates a matrix with a specified row and column block layout
    //! the blocks should be registered in the factory
    SparseBlockMatrix(const std::vector<int>& row_block_dims = std::vector<int>(),
                      const std::vector<int>& col_block_dims = std::vector<int>());

    // copies the values to dest. requires dest to have the same block structure
    void copyValues(SparseBlockMatrix& dest) const;

    // how many row blocks?
    inline int blockRows() const {
      return _row_block_dims.size();
    }

    // how many col blocks?
    inline int blockCols() const {
      return _col_block_dims.size();
    }

    // block layout (vector of size blockRows,
    // the ith component contins the row dimension of the block at row i
    inline const std::vector<int>& blockRowDims() const {
      return _row_block_dims;
    }

    // same as above, for columns
    inline const std::vector<int>& blockColDims() const {
      return _col_block_dims;
    }

    //@ returns a pair<first=rows, second=cols> containing the dimensions of the block
    inline IntPair blockDims(int block_row, int block_col) const {
      return IntPair(_row_block_dims[block_row], _col_block_dims[block_col]);
    }

    //@ returns a pair<first=rows, second=cols> containing the offset in elements of the
    // block selected as argument
    inline IntPair blockOffsets(int block_row, int block_col) const {
      return IntPair(_row_block_offsets[block_row], _col_block_offsets[block_col]);
    }

    //@ see name
    int numNonZeroBlocks() const;

    //@ non const accessor for a block
    //@ if create=true, the block is added to the structure if not present
    MatrixBlockBase* blockAt(int block_row, int block_col, bool create = false);

    //@ const accessor for a block
    const MatrixBlockBase* blockAt(int block_row, int block_col) const;

    //@ deletes a block
    void blockErase(int block_row, int block_col);

    //@ release the ownership and delete a block;
    MatrixBlockBase* blockRelease(int block_row, int block_col);

    // element operations
    // how many rows?
    inline int rows() const {
      return (!_cols.size()) ? 0 : _row_block_offsets.back();
    }

    // how many cols?
    inline int cols() const {
      return (!_cols.size()) ? 0 : _col_block_offsets.back();
    }

    // how many non zeroes, blocks are assumed to be all full for non symmetric matricies
    // while for symmetric structure only the upper triangular elements of the diagonal
    // blocks are counted
    int numNonZeros(bool is_symmetric_ = true) const;

    // manipulators
    // sets to the value of zero all allocated blocks
    // no change in structure
    void setZero();

    // allocates all blocks
    void allocateFull();

    // structure
    //! true if the block dimensions are the same as the row dimensions
    bool isLayoutSymmetric() const;

    //! true if the structure of self and other is the same (no allocation)
    bool isStructureEqual(const SparseBlockMatrix& other) const;

    //! prints stuff
    enum PrintMode { PrintFillIn, PrintDim, PrintOffset, PrintValues };
    void printLayout(PrintMode mode = PrintMode::PrintValues) const;

    //! returns the coefficients along the diagonal
    void getDiagonal(std::vector<float>& diagonal) const;

    //! sets the coefficients along the diagonal
    void setDiagonal(const std::vector<float>& diagonal);

    //! returns a dense vector
    void getDenseVector(std::vector<float>& v) const;

    // ! get the matrix in CCS format (assume upper triangular structure for symmetric matricies)
    void
    toCCS(int* row_indicies_, int* col_pointers_, double* values_, bool is_symmetric_ = true) const;

    // ! fill values in CCS matrix format (assume upper triangular structure for symmetric
    // matricies)
    void fillCCS(double* values_, bool is_simmetric_ = true) const;

  protected:
    void recomputeOffsets();

    std::vector<int> _row_block_dims;
    std::vector<int> _col_block_dims;
    std::vector<int> _row_block_offsets;
    std::vector<int> _col_block_offsets;
    IntBlockMapVector _cols;
  };

} // namespace srrg2_solver
