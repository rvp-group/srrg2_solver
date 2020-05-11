#pragma once
#include "srrg_solver/solver_core/variable.h"
#include <srrg_geometry/geometry3d.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  /** @brief SE3 Pose Variable base class.
   */
  class VariableSE3Base : public Variable_<6, Isometry3_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /** With this param one can decide whether in the perturbation process, the multiplication must
     * be performed on the right or on the left side.
     */
    enum PerturbationSide { Left = 0x1, Right = 0x2 };
    /** With this param one can decide which angle representation
     * to choose in the optimization process
     */
    enum PerturbationType { Quaternion = 0x1, Euler = 0x2 };

    virtual void setZero() override;

    void draw(ViewerCanvasPtr canvas_) const override;
  };

  /** SE3 Pose Variable specialized by the perturbation side and angle representation.
   */
  template <VariableSE3Base::PerturbationType PerturbationType_ = VariableSE3Base::Euler,
            VariableSE3Base::PerturbationSide PerturbationSide_ = VariableSE3Base::Right>
  class VariableSE3_ : public VariableSE3Base {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static const VariableSE3Base::PerturbationSide PerturbationSide = PerturbationSide_;
    static const VariableSE3Base::PerturbationType PerturbationType = PerturbationType_;
    using BaseVariableType = VariableSE3_<PerturbationType_, PerturbationSide_>;

    virtual void applyPerturbation(const Vector6f& pert) override {
      this->_tainted = true;
      typedef VariableSE3Base BaseT;
      BaseT::EstimateType pert_m;
      switch (PerturbationType) {
        case Euler:
          pert_m = geometry3d::ta2t(pert);
          break;
        case Quaternion:
          pert_m = geometry3d::v2t(pert);
      }

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

  using VariableSE3EulerRight = VariableSE3_<VariableSE3Base::Euler, VariableSE3Base::Right>;
  using VariableSE3EulerLeft  = VariableSE3_<VariableSE3Base::Euler, VariableSE3Base::Left>;
  using VariableSE3QuaternionRight =
    VariableSE3_<VariableSE3Base::Quaternion, VariableSE3Base::Right>;
  using VariableSE3QuaternionLeft =
    VariableSE3_<VariableSE3Base::Quaternion, VariableSE3Base::Left>;

} // namespace srrg2_solver
