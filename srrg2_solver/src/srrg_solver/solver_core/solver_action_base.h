#pragma once
#include <memory>
#include <set>
#include <vector>

#include <srrg_geometry/geometry_defs.h>
#include <srrg_system_utils/system_utils.h>

namespace srrg2_solver {

  class SolverBase;

  class SolverActionBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! @brief object life, you can construct an action only on a solver instance
    SolverActionBase() = delete;
    SolverActionBase(SolverBase* solver_) {
      if (solver_ == nullptr) {
        throw std::runtime_error("SolverActionBase::SolverActionBase|invalid solver pointer");
      }

      _solver_ptr = solver_;
    }

    //! @brief virtual dtor
    virtual ~SolverActionBase() {
      // ia nothin to do here
    }

    //! @brief base action that does nothin;
    virtual void doAction() {
    }

  protected:
    SolverBase* _solver_ptr = nullptr;
  };

  using SolverActionBasePtr       = std::shared_ptr<SolverActionBase>;
  using SolverActionBasePtrVector = std::vector<SolverActionBasePtr>;
  using SolverActionBasePtrSet    = std::set<SolverActionBasePtr>;

} // namespace srrg2_solver
