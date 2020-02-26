#pragma once
#include "matrix_block.h"

namespace srrg2_solver {

  // factory for blocks of fixed size
  // remember to add allocators for all blocks you might need
  // in your problem
  // use MatrixBlockFactory::instance()->addAllocator<r,c>();

  class MatrixBlockFactory {
  protected:
    static inline uint64_t hash(int rows, int cols) {
      return ((uint64_t) rows) << 32 | ((uint64_t) cols);
    }

    struct BlockAllocatorBase {
      virtual MatrixBlockBase* makeBlock() = 0;

      inline int rows() const {
        return _rows;
      }

      inline int cols() const {
        return _cols;
      }

    protected:
      int _rows = 0;
      int _cols = 0;
    };

    template <int rows_, int cols_>
    struct BlockAllocator_ : BlockAllocatorBase {
      BlockAllocator_() {
        this->_rows = rows_;
        this->_cols = cols_;
      }

      virtual MatrixBlockBase* makeBlock() override {
        return new MatrixBlock_<rows_, cols_>();
      }
    };

    using BlockAllocatorPtr = std::unique_ptr<BlockAllocatorBase>;

    using BlockAllocatorMap = std::map<uint64_t, BlockAllocatorPtr>;

    BlockAllocatorMap _allocators;
    static MatrixBlockFactory* _instance;

  public:
    template <int r, int c>
    void addAllocator() {
      uint64_t h = hash(r, c);
      if (_allocators.find(h) != _allocators.end()) {
        return;
      }
      _allocators.insert(make_pair(h, BlockAllocatorPtr(new BlockAllocator_<r, c>())));
    }

    MatrixBlockBase* createBlock(int r, int c);
    void printAllocatorsList() const;
    static MatrixBlockFactory* instance();
  };
} // namespace srrg2_solver
