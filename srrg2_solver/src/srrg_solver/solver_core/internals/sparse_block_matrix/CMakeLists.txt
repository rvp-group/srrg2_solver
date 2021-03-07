add_library(srrg2_solver_sparse_block_matrix_library SHARED
  c_matrix_ops.hpp

  matrix_block.h
  matrix_block.cpp

  matrix_block_factory.h
  matrix_block_factory.cpp

  sparse_block_matrix.h
  sparse_block_matrix.cpp

  sparse_block_cholesky.h
  sparse_block_cholesky.cpp
)
