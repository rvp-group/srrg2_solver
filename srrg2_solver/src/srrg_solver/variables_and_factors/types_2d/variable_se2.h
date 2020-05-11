#pragma once
#include "srrg_solver/solver_core/variable.h"
#include <srrg_geometry/geometry2d.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  /** @brief SE2 Pose Variable base class.
   */
  class VariableSE2Base : public Variable_<3, Isometry2_> {
  public:
    /** With this param one can decide whether in the perturbation process, the multiplication must
     * be performed on the right or on the left side.
     */
    enum PerturbationSide { Left = 0x1, Right = 0x2 };

    virtual void setZero() override {
      setEstimate(Isometry2f::Identity());
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  /** SE2 Pose Variable specialized by the perturbation side.
   */
  template <VariableSE2Base::PerturbationSide PerturbationSide_ = VariableSE2Base::Right>
  class VariableSE2_ : public VariableSE2Base {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const VariableSE2Base::PerturbationSide PerturbationSide = PerturbationSide_;
    // break here the chain of indirection
    using BaseVariableType = VariableSE2_<PerturbationSide>;

    virtual void applyPerturbation(const Vector3f& pert) override {
      typedef VariableSE2Base BaseT;
      BaseT::EstimateType pert_m = geometry2d::v2t(pert);
      this->_tainted             = true;
      switch (PerturbationSide) {
        case Left:
          _estimate = pert_m * _estimate;
          break;
        case Right:
          _estimate = _estimate * pert_m;
          break;
        default:
          assert(0);
      }
    }
  };

  using VariableSE2Right = VariableSE2_<VariableSE2Base::Right>;
  using VariableSE2Left  = VariableSE2_<VariableSE2Base::Left>;

} // namespace srrg2_solver
