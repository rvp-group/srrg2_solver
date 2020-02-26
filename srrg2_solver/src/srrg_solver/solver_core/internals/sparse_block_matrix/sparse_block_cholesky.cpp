#include "sparse_block_cholesky.h"
#include "matrix_block_factory.h"
#include <iostream>

namespace srrg2_solver {
  using namespace std;

  SparseBlockCholesky::SparseBlockCholesky(const SparseBlockMatrix& other):
    SparseBlockMatrix(other._row_block_dims, other._col_block_dims),
    _inv_Ljj(other.blockRows()){
    for (size_t c = 0; c < _cols.size(); ++c) {
      const IntBlockMap& other_col = other._cols[c];
      IntBlockMap& col = _cols[c];
      for (auto it = other_col.begin(); it != other_col.end(); ++it) {
        int row = it->first;
        MatrixBlockBase* copy = it->second.get()->clone();
        col.insert(std::make_pair(row, MatrixBlockBasePtr(copy)));
      }
    }
  }

  bool SparseBlockCholesky::structureScalarColProd(int col_idx_i, int col_idx_j) const {
    using namespace std;
    const IntBlockMap& col_i = _cols[col_idx_i];
    const IntBlockMap& col_j = _cols[col_idx_j];
    auto it_i = col_i.begin();
    auto it_j = col_j.begin();
    while (it_i != col_i.end()
           && it_j != col_j.end()
           && it_i->first < col_idx_i
           && it_j->first < col_idx_j) {
      if (it_i->first == it_j->first) {
        return true;
      }
      if (it_i->first < it_j->first) {
        ++it_i;
      } else {
        ++it_j;
      }
    }
    return false;
  }

  int SparseBlockCholesky::scalarColProd(MatrixBlockBase* dest, int col_idx_i, int col_idx_j) const {
    int ops = 0;
    using namespace std;
    const IntBlockMap& col_i = _cols[col_idx_i];
    const IntBlockMap& col_j = _cols[col_idx_j];
    auto it_i = col_i.begin();
    auto it_j = col_j.begin();
    while (it_i != col_i.end()
           && it_j != col_j.end()
           && it_i->first < col_idx_i
           && it_j->first < col_idx_j) {
      if (it_i->first == it_j->first) {
        //int col_idx=it_i->first;
        const MatrixBlockBase* block_i = it_i->second.get();
        const MatrixBlockBase* block_j = it_j->second.get();
        assert(block_i->rows() == block_j->rows() && "src col mismatch");
        assert(dest->cols() == block_j->cols() && "dest row mismatch");
        assert(dest->rows() == block_i->cols() && "dest col mismatch");
        dest->subAtxB(block_i, block_j);
        ++ops;
      }
      if (it_i->first < it_j->first) {
        ++it_i;
      } else {
        ++it_j;
      }
    }
    return ops;
  }

  void SparseBlockCholesky::cholError(ErrorType type, const char* message, int row_idx, int col_idx, const MatrixBlockBase* block) const {
    const char* s = 0;
    switch (type) {
      case ErrorFatal: s = "FATAL";
        break;
      case ErrorAbort: s = "ABORTING";
        break;
      case ErrorWarning: s = "WARNING";
        break;
      default:;
    }
    printf("Cholesky error[%s]: %s, row: %d, col %d\nBlock:\n",
           s, message, row_idx, col_idx);
    if (block) {
      block->print();
    }
    if (type == ErrorFatal) {
      throw std::runtime_error("Terminating");
    }
  }

  bool SparseBlockCholesky::choleskyAllocate() {
    assert(isLayoutSymmetric() && "layout not symmetric, cannot compute block cholesky");
    // we determine the cholesky structure
    for (int col_idx = 0; col_idx < blockRows(); ++col_idx) {
      IntBlockMap& ch_col = _cols[col_idx];
      if (ch_col.empty()) {
        return false;
      }

      auto it = ch_col.begin();
      int row_start_idx = it->first;
      //ia looping over src cols
      for (int row_idx = row_start_idx; row_idx <= col_idx; ++row_idx) {
        //IntBlockMap& ch_upper_col = _cols[row_idx];
        bool not_empty = structureScalarColProd(col_idx, row_idx);
        if (not_empty) {
          blockAt(row_idx, col_idx, true);
        }
      }
    }
    return true;
  }

  bool SparseBlockCholesky::choleskyUpdate() {
    using namespace std;
    // clear the workspace but leavce the allocation
    //setZero();

    // we allocate storage for the inverses of the diagonal blocks
    MatrixBlockFactory* factory = MatrixBlockFactory::instance();
    size_t ops = 0;
    // we run through the blocks of the allocated cholesky factor, and use the cholesky update
    for (int col_idx = 0; col_idx < blockCols(); ++col_idx) {
      IntBlockMap& col = _cols[col_idx];
      for (auto it = col.begin(); it != col.end(); ++it) {
        int row_idx = it->first;
        if (row_idx > col_idx) {
          cholError(ErrorWarning, "the matrix has some stuff in the lower part", row_idx, col_idx);
          continue;
        }
        MatrixBlockBase* src_block = it->second.get();

        ops += scalarColProd(src_block, row_idx, col_idx);
        if (col_idx == row_idx) {
          int chol_ok = src_block->llt();
          if (!chol_ok) {
            cholError(ErrorAbort, "Chol: cannot compute factor: ", row_idx, col_idx, src_block);
            return false;
          }
          const int n = src_block->rows();
          MatrixBlockBase* iljj = factory->createBlock(n, n);
          _inv_Ljj[row_idx].reset(iljj);
          bool inv_ok = src_block->inverseTo(iljj);
          if (!inv_ok) {
            cholError(ErrorAbort, "Chol: cannot compute inverse: ", row_idx, col_idx, src_block);
            return false;
          }
          iljj->transposeInPlace();
        } else {
          const MatrixBlockBase* iljj = _inv_Ljj[row_idx].get();
          src_block->scale(iljj);
        }
      }
    }
    //cerr << "computing cholesky requires :" << ops << " block operations" << endl;
    return true;
  }

  bool SparseBlockCholesky::blockCholeskySolve(IntBlockMap& x) const {
    assert(rows() == cols());

    if(x.size() > (size_t) rows()){
      throw std::runtime_error("SparseBlockCholesky::blockCholeskySolve | number of variables greater than matrix rows");
    }
    
    if(!blockForwardSubstitution(x)){
      return false;
    }

    if(!blockBackwardSubstitution(x)){
      return false;
    }

    return true;
  }

  bool SparseBlockCholesky::blockBackwardSubstitution(IntBlockMap& x) const {

    MatrixBlockFactory* factory = MatrixBlockFactory::instance();
    for(int c = blockCols()-1; c>=0; --c){
      // Extract cached inverse and transpose it
      const MatrixBlockBase* transpose_invL_cc = _inv_Ljj[c].get();
      if(!transpose_invL_cc){
        throw std::runtime_error("SparseBlockCholesky::blockBackwardSubstitution| missing diagonal block index");
      }

      MatrixBlockBase* invL_cc = factory->createBlock(transpose_invL_cc->rows(),transpose_invL_cc->cols());
      transpose_invL_cc->transposeTo(invL_cc);
      // get the corresponding row element and compute the solution for the block
      IntBlockMap::iterator row_element = x.find(c);
      if(row_element == x.end()){
        continue;
      }
      MatrixBlockBase* x_c = row_element->second.get();
      x_c->leftMatMulInPlace(invL_cc);
      // apply the result to all the other variables
      for(int r=c-1; r>=0; --r){
        const MatrixBlockBase* L_rc = blockAt(r,c);
        if(L_rc){
          IntBlockMap::iterator x_r = x.find(r);
          MatrixBlockBase* block = nullptr;
          if(x_r == x.end()){
            block = factory->createBlock(L_rc->rows(), x_c->cols());
            block->setZero();
            x.insert(make_pair(r, MatrixBlockBasePtr(block)));
          } else {
            block = x_r->second.get();
          }
          block->subAxB(L_rc,x_c);
        }
      }
      // release the memory for the transpose inverse block
      delete invL_cc;
    }
    return true;
  }

  bool SparseBlockCholesky::blockForwardSubstitution(IntBlockMap& x) const {

    MatrixBlockFactory* factory = MatrixBlockFactory::instance();
    for(int c = 0; c<blockCols(); ++c){
      // Extract the cached inverse block
      const MatrixBlockBase* invL_cc = _inv_Ljj[c].get();
      if(!invL_cc){
        throw std::runtime_error("SparseBlockCholesky::blockForwardSubstitution| missing diagonal block index");
      }
      // get the corresponding row element and compute the solution for the block
      IntBlockMap::iterator row_element = x.find(c);
      if(row_element == x.end()){
        continue;
      }
      MatrixBlockBase* x_c = row_element->second.get();
      x_c->leftMatMulInPlace(invL_cc);
      // apply the result to all the other variables
      for(int r=c+1;r<blockCols();++r){
        const MatrixBlockBase* L_cr = blockAt(c,r);
        if(L_cr){
          IntBlockMap::iterator x_r = x.find(r);
          MatrixBlockBase* block = nullptr;
          if(x_r == x.end()){
            block = factory->createBlock(L_cr->cols(), x_c->cols());
            block->setZero();
            x.insert(make_pair(r, MatrixBlockBasePtr(block)));
          } else {
            block = x_r->second.get();
          }
          block->subAtxB(L_cr,x_c);
        }
      }
    }
    return true;
  }

  bool SparseBlockCholesky::forwardSubstitute(float* x) const {
    // this is for the temporary transposition of objects
    float buffer[1024];

    // forward substitution Lt y = b
    for (int r = 0; r < blockCols(); ++r) {
      const IntBlockMap& row = _cols[r];

      // coefficient vector
      //int b_dim=_col_block_dims[r];
      float* x_r = x + _col_block_offsets[r];

      for (auto it = row.begin(); it != row.end(); ++it) {
        int col_idx = it->first;

        const MatrixBlockBase* _l = it->second.get();
        assert(_l->size() < 1024 && "temporary buffer overflow");
        MatrixBlockAlias l_block(_l->cols(), _l->rows(), buffer);
	//std::cerr << "size: " << _l->rows() << " " << _l->cols() << " " << _l->storage() <<"-> ";
	//std::cerr << "size: " << l_block.rows() << " " << l_block.cols() << " " << l_block.storage();
	_l->transposeTo(&l_block);
	//std::cerr << "ok" << endl;

        //int x_dim=_col_block_dims[col_idx];
        float* x_i = x + _col_block_offsets[col_idx];

        if (col_idx == r) {
          if (!l_block.solveTriangular(x_r, false)) {
            cholError(ErrorAbort, "undetermined linear system, forward substitution", r, it->first);
            return false;
          }

          break;
          // compute dx
        } else {
          //assert(l_block.rows() == _col_block_dims[col_idx]);
          l_block.subAxb(x_r, x_i);
        }
      }
    }
    return true;
  }

  bool SparseBlockCholesky::backSubstitute(float* x) const {

    //backward substitution Lt x= y
    for (int c = blockCols() - 1; c >= 0; --c) {
      const IntBlockMap& col = _cols[c];

      //int x_dim=_row_block_dims[c];
      float* dx = x + _row_block_offsets[c];
      bool first_round = true;

      for (auto it = col.rbegin(); it != col.rend(); ++it) {
        int row_idx = it->first;
        const MatrixBlockBase* l_block = it->second.get();
        if (first_round) {
          assert(it->first == c);
          if (!l_block->solveTriangular(dx, true)) {
            cholError(ErrorAbort, "undetermined linear system, backward substitution", c, it->first);
            return false;
          }
          // compute dx
        } else {
          float* b_start = x + _row_block_offsets[row_idx];
          //assert(l_block->rows() == _row_block_dims[row_idx]);
          l_block->subAxb(b_start, dx);
        }
        first_round = false;
      }
    }
    return true;
  }

  bool SparseBlockCholesky::choleskySolve(SparseBlockMatrix& x_) const {
    using namespace std;
    assert(rows() == cols());
    int dim = rows();

    assert(x_.cols() == 1);
    assert(x_.rows() == dim);

    //float x[dim];
    
    float* x=new float[dim];
    std::unique_ptr<float[]> x_ptr(x);

    // flatten coefficient vector
    IntBlockMap& x_col = x_._cols[0];
    for (auto it = x_col.begin(); it != x_col.end(); ++it) {
      int row_idx = it->first;
      MatrixBlockBase* b_block = it->second.get();

      int x_offset = _row_block_offsets[row_idx];
      memcpy(x + x_offset, b_block->storage(), b_block->rows() * sizeof(float));
    }

    if (!forwardSubstitute(x)) {
      return false;
    }

    if (!backSubstitute(x)) {
      return false;
    }

    for (auto it = x_col.begin(); it != x_col.end(); ++it) {
      int row_idx = it->first;
      MatrixBlockBase* b_block = it->second.get();
      int x_offset = _row_block_offsets[row_idx];
      memcpy(b_block->storage(), x + x_offset, b_block->rows() * sizeof(float));
    }
    return true;

  }

}
