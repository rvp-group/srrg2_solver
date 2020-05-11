#include "matrix_block_factory.h"
#include <iostream>

namespace srrg2_solver {
  using IntPair = std::pair<int, int>;

  MatrixBlockFactory __my_factory;

  MatrixBlockFactory* MatrixBlockFactory::instance() {
    return &__my_factory;
  }

  MatrixBlockBase* MatrixBlockFactory::createBlock(int r, int c) {
    uint64_t h = hash(r, c);
    auto it    = _allocators.find(h);
    if (it == _allocators.end()) {
      // ia if you end up here you probably have to add your allocator in the SparseSolver ctor
      std::cerr
        << "MatrixBlockFactory::createBlock|please create an allocator in for blocks of dimension <"
        << r << ", " << c << ">";
      assert(0 && "you did not create an allocator for a block" && r && c);
      return 0;
    }
    return it->second->makeBlock();
  }

  void MatrixBlockFactory::printAllocatorsList() const {
    using namespace std;
    cerr << "MatrixBlockFactory, list of allocators: " << endl;
    for (auto it = _allocators.begin(); it != _allocators.end(); ++it) {
      cerr << " hash " << it->first << " rows: " << it->second->rows()
           << " cols: " << it->second->cols() << endl;
    }
  }

} // namespace srrg2_solver
