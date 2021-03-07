add_library(srrg2_solver_linear_solvers_library SHARED

  sparse_block_linear_solver.cpp
  sparse_block_linear_solver.h
  
  sparse_block_linear_solver_cholesky.cpp
  sparse_block_linear_solver_cholesky.h 

  quotient_graph.cpp
  quotient_graph.h
  
  sparse_block_linear_solver_cholesky_emd.cpp
  sparse_block_linear_solver_cholesky_emd.h 

  sparse_block_linear_solver_cholesky_csparse.cpp
  sparse_block_linear_solver_cholesky_csparse.h 

  sparse_block_linear_solver_cholesky_cholmod.cpp
  sparse_block_linear_solver_cholesky_cholmod.h 

  sparse_block_linear_solver_cholmod_full.cpp
  sparse_block_linear_solver_cholmod_full.h 

  sparse_block_linear_solver_qr.cpp
  sparse_block_linear_solver_qr.h 

  instances.h
  instances.cpp
  )

target_link_libraries(srrg2_solver_linear_solvers_library
  srrg2_solver_sparse_block_matrix_library
  ${CHOLMOD_LIBRARIES}
  ${UMFPACK_LIBRARIES}
  ${CSPARSE_LIBRARY}
  )
