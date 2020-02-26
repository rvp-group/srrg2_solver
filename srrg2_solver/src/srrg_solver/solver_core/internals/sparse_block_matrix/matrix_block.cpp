#include "matrix_block.h"
#include "vectorized_matrix_ops.hpp"

namespace srrg2_solver {

  using Ops = VectorizedMatrixOps_<1, 2, 3, 6>;
  Ops ops;

  MatrixBlockBase::~MatrixBlockBase() {
  }

  bool MatrixBlockBase::solveTriangular(float* x, bool upper) const {
    return ops.cMatSolveTriangular(x, _storage, _rows, upper) == 0;
  }

  void MatrixBlockBase::copyTo(MatrixBlockBase* other) const {
    assert(other->cols() == cols() && other->rows() == rows());
    ops.cMatCopy(other->_storage, _storage, _rows, _cols);
  }

  void MatrixBlockBase::print() const {
    ops.cMatPrint(stdout, _storage, _rows, _cols);
  }

  void MatrixBlockBase::transposeTo(MatrixBlockBase* dest) const {
    assert(dest->rows() == cols() && dest->cols() == rows());
    ops.cMatTranspose(dest->_storage, _storage, _rows, _cols);
  }

  void MatrixBlockBase::scale(const MatrixBlockBase* scale) {
    assert(scale->rows() == scale->cols());
    assert(scale->rows() == rows());
    ops.cMatScaleInPlace(_storage, scale->_storage, scale->rows(), cols());
  }

  void MatrixBlockBase::transposeInPlace() {
    ops.cMatTransposeInPlace(_storage, _rows);
  }

  bool MatrixBlockBase::llt() {
    assert(rows() == cols());
    return ops.cMatLLt(_storage, rows()) == 0;
  }

  bool MatrixBlockBase::inverseTo(const MatrixBlockBase* dest) const {
    assert(rows() == cols());
    assert(dest->rows() == rows() && dest->cols() == cols());
    return ops.cMatInvert(dest->_storage, _storage, _rows, 0) == 0;
  }

  void MatrixBlockBase::subAtxB(const MatrixBlockBase* a, const MatrixBlockBase* b) {
    assert(a->rows() == b->rows());
    assert(rows() == a->cols());
    assert(cols() == b->cols());
    ops.cMatProdATransposeBNegate(
      _storage, a->_storage, b->_storage, a->cols(), b->cols(), a->rows());
  }

  void MatrixBlockBase::subAxB(const MatrixBlockBase* a, const MatrixBlockBase* b) {
    assert(a->cols() == b->rows());
    assert(rows() == a->rows());
    assert(cols() == b->cols());
    ops.cMatProdABNegate(_storage, a->_storage, b->_storage, a->rows(), a->cols(), b->cols());
  }

  void MatrixBlockBase::subAxb(float* dest, const float* x) const {
    ops.cMatVecProdSub(dest, _storage, x, _rows, _cols);
  }

  void MatrixBlockBase::leftMatMulInPlace(const MatrixBlockBase* x) {
    assert(x->rows() == x->cols());
    assert(x->cols() == rows());
    ops.cMatMatLeftProdInPlace(_storage, x->storage(), _rows, _cols);
  }

  void MatrixBlockBase::rightMatMulInPlace(const MatrixBlockBase* x) {
    assert(x->rows() == x->cols());
    assert(x->rows() == cols());
    ops.cMatMatRightProdInPlace(_storage, x->storage(), _rows, _cols);
  }

  void MatrixBlockBase::setZero() {
    int dim = _rows * _cols;
    memset(_storage, 0, dim * sizeof(float));
  }

  void MatrixBlockBase::setIdentity() {
    assert(rows() == cols());
    cMatSetIdentity(_storage, rows());
  }

} // namespace srrg2_solver
