#include <numeric>
#include <srrg_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholmod_full.h>
#include <srrg_solver/solver_core/solver.h>
#include <srrg_solver/solver_core/internals/sparse_block_matrix/matrix_block.h>
#include <srrg_solver/solver_core/internals/sparse_block_matrix/matrix_block_factory.h>
#include <srrg_solver/solver_core/internals/sparse_block_matrix/vectorized_matrix_ops.hpp>
#include <srrg_system_utils/system_utils.h>
#include <suitesparse/cholmod.h>
using namespace srrg2_core;
using namespace srrg2_solver;
using namespace std;

int main(int argc, char** argv) {
  int block_size[] = {3, 3, 3, 3};
  float diag_value = 10;

  int n                       = sizeof(block_size) / sizeof(int);
  MatrixBlockFactory* factory = MatrixBlockFactory::instance();
  Solver slv;
  vector<int> block_dims(block_size, block_size + n);
  int num_elements_diag = accumulate(block_dims.begin(), block_dims.end(), 0);
  SparseBlockMatrix A(block_dims, block_dims);

  vector<float> diag(num_elements_diag, diag_value);
  A.setDiagonal(diag);
  MatrixBlockBase* diagonal = A.blockAt(0, 0, true);
  cVecFillRandom(diagonal->storage(), diagonal->rows() * diagonal->cols());
  cMatTriangularize(diagonal->storage(), diagonal->rows(), diagonal->cols());
  MatrixBlockBase* block = factory->createBlock(diagonal->rows(), diagonal->cols());
  diagonal->transposeTo(block);
  cVecSum(diagonal->storage(), block->storage(), diagonal->rows() * diagonal->cols());
  MatrixBlockBase* offdiagonal_block1 = A.blockAt(0, 2, true);
  offdiagonal_block1->setIdentity();
  cVecSum(diagonal->storage(), offdiagonal_block1->storage(), diagonal->rows() * diagonal->cols());

  A.printLayout();
  cholmod_sparse* chol;
  cholmod_dense *x, *b;
  cholmod_factor* L;
  /* basic scalars */
  cholmod_common c;
  cholmod_start(&c);
  c.final_ll   = 1;
  c.supernodal = CHOLMOD_SUPERNODAL;
  /* start CHOLMOD */
  chol = cholmod_allocate_sparse(A.rows(), A.cols(), A.numNonZeros(), 1, 1, 1, CHOLMOD_REAL, &c);
  size_t rows = A.rows();
  chol->p     = new int[rows + 1];
  chol->i     = new int[chol->nzmax];
  chol->x     = new double[chol->nzmax];
  A.toCCS((int*) chol->i, (int*) chol->p, (double*) chol->x);
  A.fillCCS((double*) chol->x);
  /* read in a matrix */
  cholmod_print_sparse(chol, "A", &c);
  /* print the matrix */
  b = cholmod_zeros(A.rows(), 1, CHOLMOD_REAL, &c);
  // b->dtype = CHOLMOD_SINGLE;
  std::vector<double> v(A.rows(), 1);
  b->x = v.data();
  /* b = ones(n,1) */
  
  int* null_permutation = new int[A.rows()];
  std::unique_ptr<int[]> null_permutation_ptr(null_permutation);
  
  c.nmethods           = 0;
  c.method[0].ordering = CHOLMOD_GIVEN;
  std::iota(null_permutation, null_permutation + A.rows(), 0);
  L        = cholmod_analyze_p(chol, null_permutation, NULL, 0, &c);
  L->is_ll = 1;

  /* analyze */
  cholmod_factorize(chol, L, &c);
  cholmod_print_factor(L, "L", &c);
  /* factorize */
  x = cholmod_solve(CHOLMOD_A, L, b, &c);

  double* data = (double*) x->x;
  for (size_t i = 0; i < chol->nrow; ++i) {
    std::cerr << "Pure Cholmod : x[" << i << "] = " << data[i] << std::endl;
  }
  cholmod_print_dense(x, "cazzo", &c);
  cerr << "Set target block" << endl;
  SparseBlockMatrix B(block_dims, std::vector<int>(1, 1));

  block           = B.blockAt(0, 0, true);
  block->at(0, 0) = 1.0f;
  block->at(1, 0) = 1.0f;
  block->at(2, 0) = 1.0f;

  block           = B.blockAt(1, 0, true);
  block->at(0, 0) = 1.0f;
  block->at(1, 0) = 1.0f;
  block->at(2, 0) = 1.0f;

  block           = B.blockAt(2, 0, true);
  block->at(0, 0) = 1.0f;
  block->at(1, 0) = 1.0f;
  block->at(2, 0) = 1.0f;

  block           = B.blockAt(3, 0, true);
  block->at(0, 0) = 1.0f;
  block->at(1, 0) = 1.0f;
  block->at(2, 0) = 1.0f;
  B.printLayout();

  SparseBlockLinearSolverCholmodFull solver;
  solver.bindLinearSystem(&A, &B);
  cerr << "Set target block" << endl;
  SystemUsageCounter::tic();
  solver.compute();
  double time = SystemUsageCounter::toc();
  std::cerr << " Time : " << time << std::endl;
  const SparseBlockMatrix& X = solver.x();
  X.printLayout();
}
