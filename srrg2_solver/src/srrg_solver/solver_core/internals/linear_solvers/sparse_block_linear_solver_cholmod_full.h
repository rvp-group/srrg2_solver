#pragma once
#include "sparse_block_linear_solver.h"
#include <suitesparse/cholmod.h>

namespace srrg2_solver {

  class SparseBlockLinearSolverCholmodFull : public SparseBlockLinearSolver {
  public:
    // @brief setup cholmod parameters
    SparseBlockLinearSolverCholmodFull();

    // @brief Free cholmod structures
    ~SparseBlockLinearSolverCholmodFull();

    // @brief TODO: block inversion
    bool computeBlockInverse(SparseBlockMatrix& inverse_blocks) override {
      return false;
    };

    // @brief compute matrix permutation based on block structure
    void computeOrderingHint(std::vector<int>& ordering,
                             const std::vector<IntPair>& block_layout) const override;

  protected:
    // computes the internal structure, given the structure of A
    // @returns: the status (StructureGood or StructureBad)
    Status updateStructure() override;

    // copies the coefficients, and updates the numerical
    // structure for the solver
    // assumes structure is ok
    // @returns: the status (CoefficientsGood or CoefficientsBad)
    Status updateCoefficients() override;

    // solves the linear system based on the updated coefficients
    // assumes coefficients are ok
    // @returns: the status (SolutionGood or SolutionBad)
    Status updateSolution() override;

    // tg name are choosen in order to not collapse with mother class attributes
    // @brief parameter settings, statistics, and workspace used internally by cholmod
    cholmod_common _cholmodCommon;
    
    // tg i use NULL to make it consistent with the output of cholmod_free_stuff 
    // @brief system matrix in cholmod sparse format
    cholmod_sparse* _B = NULL;
    // @brief target vector in cholmod dense format
    cholmod_dense* _c = NULL;
    // @brief solution vector in cholmod dense format
    cholmod_dense* _solution = NULL;
    // @brief Cholesky factor in cholmod format
    cholmod_factor* _L = NULL;
  };
} // namespace srrg2_solver