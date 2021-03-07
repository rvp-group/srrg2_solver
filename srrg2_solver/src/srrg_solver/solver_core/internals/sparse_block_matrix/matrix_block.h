#pragma once
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <map>
#include <memory>
#include <vector>

namespace srrg2_solver {
  using IntPair = std::pair<int, int>;

  //! base element of a block matrix
  //! implements some trivial operations
  struct MatrixBlockBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MatrixBlockBase(int rows_ = 0, int cols_ = 0) : _rows(rows_), _cols(cols_) {
    }

    virtual ~MatrixBlockBase();

    //! returns a deep copy of self
    virtual MatrixBlockBase* clone() const = 0;

    inline float& at(int r, int c) {
      return _storage[c * _rows + r];
    }

    inline float at(int r, int c) const {
      return _storage[c * _rows + r];
    }

    //! number of elements
    inline int size() const {
      return _rows * _cols;
    }

    //! number of rows
    inline int rows() const {
      return _rows;
    }

    //! number of cols
    inline int cols() const {
      return _cols;
    }

    //! pointer to the stored data
    inline float* storage() {
      return _storage;
    }

    //! pointer to stored data
    inline const float* storage() const {
      return _storage;
    }

    //! rudely prints block on screen
    void print() const;

    //! copies the values of self in the other block
    //! sizes must match
    void copyTo(MatrixBlockBase* other) const;

    //! self = scale * self
    //! scale is a matrix of proper dimensions
    void scale(const MatrixBlockBase* scale);

    //! self *= scale
    //! scale is a scalar
    void scale(const float& scale);

    //! dest = self.transpose
    void transposeTo(MatrixBlockBase* dest) const;

    //! in place transpose. self should be square
    void transposeInPlace();

    //! solves a triangular sistem
    //! self * x = b
    //! x contains the coefficients, and is overwritten with the solution
    //! upper/lower tells if the system is upper or lower triangular
    //! upper = true means upper triangular
    //! @returns true if solution ok
    bool solveTriangular(float* x, bool upper) const;

    //! computes cholesky decomposition
    //! considers only the upper part of the matrix
    //! overwrites self
    //! @returns false on failure
    bool llt();

    // dest = self.inverse()
    // @returns false on failure
    bool inverseTo(const MatrixBlockBase* dest) const;

    // x -= dest * x
    void subAxb(float* dest, const float* x) const;
    
    // self -= a^T * b // used in cholesky
    void subAtxB(const MatrixBlockBase* a, const MatrixBlockBase* b);

    // self -= a * b 
    void subAxB(const MatrixBlockBase* a, const MatrixBlockBase* b);

    // self = x * self (x must be square)
    void leftMatMulInPlace(const MatrixBlockBase* x);
    
    // self *= x (x must be square)
    void rightMatMulInPlace(const MatrixBlockBase* x);

    // out = self * x
    void matrixProduct(MatrixBlockBase* out, const MatrixBlockBase* x);

    // zeroes all values
    void setZero();

    // set the block to the identity
    void setIdentity();

    template <typename EigenType_>
    inline Eigen::Map<EigenType_> eigenType() {
      if (EigenType_::RowsAtCompileTime < 0 || EigenType_::ColsAtCompileTime < 0) {
        throw std::runtime_error("MatrixBlock| cannot cast to dynamic eigen type");
      }
      if (EigenType_::RowsAtCompileTime != _rows || EigenType_::ColsAtCompileTime != _cols) {
        throw std::runtime_error("MatrixBlock| size mismatch");
      }
      return Eigen::Map<EigenType_>(_storage);
    }

  protected:
    float* _storage = 0;
    int _rows;
    int _cols;
  };

  using MatrixBlockBasePtr = std::unique_ptr<MatrixBlockBase>;

  //! simple alias constructed on an existing matrix block
  //! on an external storage
  struct MatrixBlockAlias : public MatrixBlockBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MatrixBlockAlias(int rows_, int cols_, float* storage_) : MatrixBlockBase(rows_, cols_) {
      _storage = storage_;
    }

    // don't clone it
    virtual MatrixBlockBase* clone() const {
      return 0;
    }

    // ctor: makes the alias
    MatrixBlockAlias(const MatrixBlockBase& other, float* storage_) :
      MatrixBlockBase(other.rows(), other.cols()) {
      _storage = storage_;
      other.copyTo(this);
    }
  };

  //! base block of fixed size that uses an eigen matrix
  template <int rows_, int cols_>
  class MatrixBlock_ : public MatrixBlockBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MatrixBlock_() : MatrixBlockBase(rows_, cols_) {
      _storage = _matrix.data();
    }

    virtual MatrixBlockBase* clone() const {
      MatrixBlock_<rows_, cols_>* copy = new MatrixBlock_<rows_, cols_>();
      copy->_matrix                    = _matrix;
      return copy;
    }

    using MatrixType = typename Eigen::Matrix<float, rows_, cols_>;
    MatrixType _matrix;
  };

} // namespace srrg2_solver
