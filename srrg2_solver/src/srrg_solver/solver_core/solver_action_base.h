#pragma once
#include <memory>
#include <set>
#include <vector>

#include <srrg_geometry/geometry_defs.h>
#include <srrg_system_utils/system_utils.h>

namespace srrg2_solver {

  class SolverBase;
  /*! @brief Base solver action interface, might be pre or post a solver iteration. In the derived
    class you need to override the doAction() method */
  class SolverActionBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SolverActionBase() = delete;
    /*! Object life, you can construct an action only on a solver instance
      @param[in] solver_ pointer to solver at which the actions is assigned
     */
    SolverActionBase(SolverBase* solver_) {
      if (solver_ == nullptr) {
        throw std::runtime_error("SolverActionBase::SolverActionBase|invalid solver pointer");
      }
      _solver_ptr = solver_;
    }

    virtual ~SolverActionBase() {
    }

    /*! Perform the action */
    virtual void doAction() {
    }

  protected:
    SolverBase* _solver_ptr = nullptr; /*!< Pointer to the solver at which the action is assigned */
  };

  using SolverActionBasePtr =
    std::shared_ptr<SolverActionBase>; /*!< Shared pointer to SolverActionBase */
  using SolverActionBasePtrVector = std::vector<SolverActionBasePtr>;
  using SolverActionBasePtrSet    = std::set<SolverActionBasePtr>;

} // namespace srrg2_solver
