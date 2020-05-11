#pragma once
#include "srrg_solver/solver_core/internals/sparse_block_matrix/sparse_block_matrix.h"
#include <srrg_config/configurable.h>

namespace srrg2_solver {
  using IntPair = std::pair<int, int>;
  // mother of all lineas solvers
  // usage:
  // 1. bind matrix and coefficient vector
  //
  // 2. call init()
  //    the call should be preceded by setStructureChanged()
  //    each time the fill in layout of A changes
  //
  // 3. call compute();
  //    each call of compute should be preceded by
  //    setCoefficientsChanged(), if the numbers in A
  //    (not the structure) changed

  class SparseBlockLinearSolver : public srrg2_core::Configurable {
    template <typename Type_>
    friend struct ParamProcessingConfigurableUniquePtr_;

  public:
    enum Status {
      Error            = 0,
      SolutionGood     = 1,
      SolutionBad      = 2,
      StructureBad     = 3,
      StructureGood    = 4,
      CoefficientsBad  = 5,
      CoefficientsGood = 6
    };
    static const char* _string_status[];

    // returns a block ordering
    // in the ordering vector, resized as the dimension of the states
    // from the block layout specified in the other variable
    // block_layout:    row,column pairs, column major ordering
    virtual void computeOrderingHint(std::vector<int>& ordering,
                                     const std::vector<IntPair>& block_layout) const;

    // returns the solution
    inline const SparseBlockMatrix& x() const {
      return _x;
    }

    void setX(const SparseBlockMatrix& x_);

    SparseBlockLinearSolver();

    /**** INPUTS ****/
    // system matrix
    void bindLinearSystem(SparseBlockMatrix* A, SparseBlockMatrix* b) {
      _A                    = A;
      _b                    = b;
      _coefficients_changed = true;
      _structure_changed    = true;
    }
    /**** INPUTS  ****/

    // call this to tell solver that structure in A changed
    inline void setStructureChanged() {
      _structure_changed = true;
      _status            = Error;
    }

    // setting this to true, that values in A changed
    inline void setCoefficientsChanged() {
      _coefficients_changed = true;
    }

    // to be called to retrieve the solution (x updated)
    virtual void compute();

    Status status() const {
      return _status;
    }

    const char* stringStatus() const {
      return _string_status[_status];
    }

    // compute the inverse of specific blocks of A
    virtual bool computeBlockInverse(SparseBlockMatrix& inverse_blocks,
                                     const std::vector<IntPair>& blocks_layout) = 0;

  protected:
    // computes the internal structure, given the structure of A
    // @returns: the status (StructureGood or StructureBad)
    virtual Status updateStructure() = 0;

    // copies the coefficients, and updates the numerical
    // structure for the solver
    // assumes structure is ok
    // @returns: the status (CoefficientsGood or CoefficientsBad)
    virtual Status updateCoefficients() = 0;

    // solves the linear system based on the updated coefficients
    // assumes coefficients are ok
    // @returns: the status (SolutionGood or SolutionBad)
    virtual Status updateSolution() = 0;

    Status _status             = Error;
    bool _coefficients_changed = true;
    bool _structure_changed    = true;
    std::vector<int> _last_ordering;
    SparseBlockMatrix* _A = 0;
    SparseBlockMatrix* _b = 0;
    SparseBlockMatrix _x;
  };

  using SparseBlockLinearSolverPtr = std::shared_ptr<SparseBlockLinearSolver>;
} // namespace srrg2_solver
