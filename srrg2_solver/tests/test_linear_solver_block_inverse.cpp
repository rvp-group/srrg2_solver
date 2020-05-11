#include <numeric>
#include <srrg_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholesky_cholmod.h>
#include <srrg_solver/solver_core/internals/sparse_block_matrix/matrix_block.h>
#include <srrg_solver/solver_core/internals/sparse_block_matrix/matrix_block_factory.h>
#include <srrg_solver/solver_core/internals/sparse_block_matrix/vectorized_matrix_ops.hpp>
#include <srrg_solver/solver_core/solver.h>
#include <srrg_system_utils/system_utils.h>

#include <gtest/gtest.h>

#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>
using namespace srrg2_core;
using namespace srrg2_solver;
using namespace Eigen;
using namespace std;

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(MARGINAL_COVARIANCE, SparseBlockLinearSolverCholmodFull) {
  // Define block sizes
  int block_size[] = {3, 3, 6};
  // Value on the diagonal
  float diag_value = 10;
  // Number of blocks
  int n = sizeof(block_size) / sizeof(int);
  // Required to register block allocation
  Solver slv;
  // Define dimensions and num elements on the diagonal
  vector<int> block_dims(block_size, block_size + n);
  int num_elements_diag = accumulate(block_dims.begin(), block_dims.end(), 0);
  // Allocate matrix
  SparseBlockMatrix A(block_dims, block_dims);
  for (int rr = 0; rr < n; ++rr) {
    MatrixBlockBase* diag_block = A.blockAt(rr, rr, true);
    diag_block->setZero();
  }
  // Set value on the diagonal and off-diagonal block (matrix is assumed symmetric)
  vector<float> diag(num_elements_diag, diag_value);
  A.setDiagonal(diag);
  MatrixBlockBase* offdiagonal_block = A.blockAt(1, 2, true);
  offdiagonal_block->setZero();
  offdiagonal_block->at(0, 3) = 0.766f;
  offdiagonal_block->at(1, 4) = 1.612f;
  offdiagonal_block->at(2, 5) = -0.25f;

  // Allocate void target vector
  SparseBlockMatrix b(block_dims, std::vector<int>(1, 1));
  // Matrix where the inverse block will be written
  SparseBlockMatrix x(block_dims, block_dims);
  // Required block inversion for specific blocks
  std::vector<IntPair> blocks{IntPair(1, 1), IntPair(1, 2)};
  // Call compute (useless but required to factorize system matrix)
  SparseBlockLinearSolverPtr solver = slv.param_linear_solver.value();
  solver->bindLinearSystem(&A, &b);
  solver->compute();
  // Evaluate time and compute block inverse for the required blocks
  SystemUsageCounter sc;
  sc.tic();
  if (!solver->computeBlockInverse(x, blocks)) {
    std::cerr << "Unable to compute block inverse" << std::endl;
  }
  double time = sc.toc();
  std::cerr << FG_YELLOW(" Time : ") << time << std::endl;
  MatrixBlockBase* block = x.blockAt(1, 1);
  Matrix3f A11           = block->eigenType<Matrix3f>();

  ASSERT_FLOAT_EQ(A11(0, 0), 0.10059022);
  ASSERT_FLOAT_EQ(A11(1, 1), 0.10266787);
  ASSERT_FLOAT_EQ(A11(2, 2), 0.10006254);

  block                   = x.blockAt(1, 2);
  Matrix<float, 3, 6> A12 = block->eigenType<Matrix<float, 3, 6>>();

  ASSERT_FLOAT_EQ(A12(0, 3), -0.0077052107);
  ASSERT_FLOAT_EQ(A12(1, 4), -0.01655006);
  ASSERT_FLOAT_EQ(A12(2, 5), 0.0025015634);
}

TEST(MARGINAL_COVARIANCE, SparseBlockLinearSolverCholesky) {
  // Define block sizes
  int block_size[] = {3, 3, 6};
  // Value on the diagonal
  float diag_value = 10;
  // Number of blocks
  int n = sizeof(block_size) / sizeof(int);
  // Required to register block allocation
  Solver slv;
  // Define dimensions and num elements on the diagonal
  vector<int> block_dims(block_size, block_size + n);
  int num_elements_diag = accumulate(block_dims.begin(), block_dims.end(), 0);
  // Allocate matrix
  SparseBlockMatrix A(block_dims, block_dims);
  for (int rr = 0; rr < n; ++rr) {
    MatrixBlockBase* diag_block = A.blockAt(rr, rr, true);
    diag_block->setZero();
  }
  // Set value on the diagonal and off-diagonal block (matrix is assumed symmetric)
  vector<float> diag(num_elements_diag, diag_value);
  A.setDiagonal(diag);
  MatrixBlockBase* offdiagonal_block = A.blockAt(1, 2, true);
  offdiagonal_block->setZero();
  offdiagonal_block->at(0, 3) = 0.766f;
  offdiagonal_block->at(1, 4) = 1.612f;
  offdiagonal_block->at(2, 5) = -0.25f;

  // Allocate void target vector
  SparseBlockMatrix b(block_dims, std::vector<int>(1, 1));
  // Matrix where the inverse block will be written
  SparseBlockMatrix x(block_dims, block_dims);
  // Required block inversion for specific blocks
  std::vector<IntPair> blocks{IntPair(1, 1), IntPair(1, 2)};
  // Call compute (useless but required to factorize system matrix)
  SparseBlockLinearSolverPtr solver(new SparseBlockLinearSolverCholesky);
  solver->bindLinearSystem(&A, &b);
  solver->compute();
  // Evaluate time and compute block inverse for the required blocks
  SystemUsageCounter sc;
  sc.tic();
  if (!solver->computeBlockInverse(x, blocks)) {
    std::cerr << "Unable to compute block inverse" << std::endl;
  }
  double time = sc.toc();
  std::cerr << FG_YELLOW(" Time : ") << time << std::endl;
  MatrixBlockBase* block = x.blockAt(1, 1);
  Matrix3f A11           = block->eigenType<Matrix3f>();

  ASSERT_FLOAT_EQ(A11(0, 0), 0.10059022);
  ASSERT_FLOAT_EQ(A11(1, 1), 0.10266787);
  ASSERT_FLOAT_EQ(A11(2, 2), 0.10006254);

  block                   = x.blockAt(1, 2);
  Matrix<float, 3, 6> A12 = block->eigenType<Matrix<float, 3, 6>>();

  ASSERT_FLOAT_EQ(A12(0, 3), -0.0077052107);
  ASSERT_FLOAT_EQ(A12(1, 4), -0.01655006);
  ASSERT_FLOAT_EQ(A12(2, 5), 0.0025015634);
}
