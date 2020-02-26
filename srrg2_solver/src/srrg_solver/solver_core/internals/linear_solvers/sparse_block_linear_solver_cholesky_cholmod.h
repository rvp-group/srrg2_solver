#include "sparse_block_linear_solver_cholesky.h"

namespace srrg2_solver {

  class SparseBlockLinearSolverCholeskyCholmod : public SparseBlockLinearSolverCholesky {
  public:
    virtual void computeOrderingHint(std::vector<int>& ordering,
                                     const std::vector<IntPair>& block_layout) const;
  };
} // namespace srrg2_solver
