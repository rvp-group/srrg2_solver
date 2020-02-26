#include <srrg_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholesky_emd.h>
#include <srrg_solver/solver_core/internals/sparse_block_matrix/matrix_block_factory.h>
#include <srrg_solver/solver_core/solver.h>
#include <srrg_solver/solver_core/internals/sparse_block_matrix/matrix_block.h>
#include <srrg_solver/solver_core/internals/sparse_block_matrix/vectorized_matrix_ops.hpp>
#include <srrg_system_utils/system_utils.h>
#include <numeric>

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace std;

int main(int argc, char** argv){
  
  int block_size[] = {3,3,6,6};
  float diag_value = 10;

  int n = sizeof(block_size)/sizeof(int);
  MatrixBlockFactory* factory = MatrixBlockFactory::instance();
  Solver slv;
  vector<int> block_dims(block_size, block_size + n);
  int num_elements_diag = accumulate(block_dims.begin(),block_dims.end(),0);
  SparseBlockMatrix A(block_dims,block_dims);

  vector<float> diag( num_elements_diag, diag_value);
  A.setDiagonal(diag);
  MatrixBlockBase* diagonal = A.blockAt(0,0,true);
  cVecFillRandom(diagonal->storage(),diagonal->rows()*diagonal->cols());
  cMatTriangularize(diagonal->storage(),diagonal->rows(),diagonal->cols());
  MatrixBlockBase* block = factory->createBlock(diagonal->rows(),diagonal->cols());
  diagonal->transposeTo(block);
  cVecSum(diagonal->storage(),block->storage(),diagonal->rows()*diagonal->cols());
  MatrixBlockBase* offdiagonal_block1 = A.blockAt(0,2, true);
  offdiagonal_block1->setIdentity();
  cVecSum(diagonal->storage(),offdiagonal_block1->storage(),diagonal->rows() * diagonal->cols());

  A.printLayout(SparseBlockMatrix::PrintMode::PrintValues);
  cerr << "Set target block" << endl;
  SparseBlockMatrix b(block_dims,block_dims);

  SystemUsageCounter sc;

  SparseBlockMatrix x(block_dims,block_dims);
  block = x.blockAt(2,2,true);
  block->setIdentity();

  block = x.blockAt(2,1,true);
  block->setZero();

  SparseBlockLinearSolverPtr solver = slv.param_linear_solver.value();
  solver->bindLinearSystem(&A,&b);
  solver->compute();
  sc.tic();
  solver->computeBlockInverse(x);
  double time = sc.toc();
  std::cerr << " Time : " << time << std::endl;
  x.printLayout(SparseBlockMatrix::PrintMode::PrintValues);
}

  
