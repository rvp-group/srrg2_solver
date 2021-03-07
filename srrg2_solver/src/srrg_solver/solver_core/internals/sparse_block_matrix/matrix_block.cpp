#include "matrix_block.h"
#include "c_matrix_ops.hpp"

namespace srrg2_solver {

  MatrixBlockBase::~MatrixBlockBase() {
  }

  bool MatrixBlockBase::solveTriangular(float* x, bool upper) const {
    return cMatSolveTriangular(x, _storage, upper, _rows) == 0;
  }

  void MatrixBlockBase::copyTo(MatrixBlockBase* other) const {
    assert(other->cols() == cols() && other->rows() == rows());
    cMatCopy(other->_storage, _storage, _rows, _cols);
  }

  void MatrixBlockBase::print() const {
    cMatPrint(stdout, _storage, _rows, _cols);
  }

  void MatrixBlockBase::transposeTo(MatrixBlockBase* dest) const {
    assert(dest->rows() == cols() && dest->cols() == rows());
    cMatTranspose(dest->_storage, _storage, _rows, _cols);
  }

  void MatrixBlockBase::scale(const MatrixBlockBase* scale) {
    assert(scale->rows() == scale->cols());
    assert(scale->rows() == rows());
    cMatScaleInPlace(_storage, scale->_storage, scale->rows(), cols());
  }

  void MatrixBlockBase::scale(const float& scale) {
    int size = rows() * cols();
    cVecScale(_storage, size, scale);
  }

  void MatrixBlockBase::transposeInPlace() {
    cMatTransposeInPlace(_storage, _rows);
  }

  bool MatrixBlockBase::llt() {
    assert(rows() == cols());
    return cMatLLt(_storage, rows()) == 0;
  }

  bool MatrixBlockBase::inverseTo(const MatrixBlockBase* dest) const {
    assert(rows() == cols());
    assert(dest->rows() == rows() && dest->cols() == cols());
    return cMatInvert(dest->_storage, _storage, _rows) == 0;
  }

  void MatrixBlockBase::subAtxB(const MatrixBlockBase* a, const MatrixBlockBase* b) {
    assert(a->rows() == b->rows());
    assert(rows() == a->cols());
    assert(cols() == b->cols());
    cMatProdATransposeBNegate(
      _storage, a->_storage, b->_storage, a->cols(), b->cols(), a->rows());
  }

  void MatrixBlockBase::subAxB(const MatrixBlockBase* a, const MatrixBlockBase* b) {
    assert(a->cols() == b->rows());
    assert(rows() == a->rows());
    assert(cols() == b->cols());
    cMatProdABNegate(_storage, a->_storage, b->_storage, a->rows(), a->cols(), b->cols());
  }

  void MatrixBlockBase::subAxb(float* dest, const float* x) const {
    cMatVecProdSub(dest, _storage, x, _rows, _cols);
  }

  void MatrixBlockBase::leftMatMulInPlace(const MatrixBlockBase* x) {
    assert(x->rows() == x->cols());
    assert(x->cols() == rows());
    cMatMatLeftProdInPlace(_storage, x->storage(), _rows, _cols);
  }

  void MatrixBlockBase::rightMatMulInPlace(const MatrixBlockBase* x) {
    assert(x->rows() == x->cols());
    assert(x->rows() == cols());
    cMatMatRightProdInPlace(_storage, x->storage(), _rows, _cols);
  }

  void MatrixBlockBase::matrixProduct(MatrixBlockBase* out, const MatrixBlockBase* x) {
    assert(out->rows() == rows());
    assert(out->cols() == x->cols());
    assert(cols() == x->rows());
    cMatMatProd(out->storage(), _storage, rows(), cols(), x->storage(), x->rows(), x->cols());
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
