#pragma once
#include <srrg_solver/solver_core/variable.h>

#include <srrg_geometry/geometry3d.h>
#include <srrg_geometry/similiarity.hpp>

namespace srrg2_solver {
  using namespace srrg2_core;

  class VariableSim3Base : public VariableGeneric_<7, Similiarity3_> {
  public:
    enum PerturbationSide { Left = 0x1, Right = 0x2 };
    enum PerturbationType { Quaternion = 0x1, Euler = 0x2 };

    void setZero() override {
      _estimate.setIdentity();
    }

    void draw(ViewerCanvasPtr canvas_) const override {
      if (!canvas_)
        throw std::runtime_error("VariableSim3Base::draw|invalid canvas");
      canvas_->pushColor();
      canvas_->setColor(srrg2_core::ColorPalette::color3fBlue());
      canvas_->pushMatrix();
      canvas_->multMatrix(_estimate.matrix());
      //      canvas_->putSphere(0.1);
      canvas_->putReferenceSystem(1);
      canvas_->popMatrix();
      canvas_->popAttribute();
    }
  };

  template <VariableSim3Base::PerturbationType PerturbationType_ = VariableSim3Base::Euler,
            VariableSim3Base::PerturbationSide PerturbationSide_ = VariableSim3Base::Right>
  class VariableSim3_ : public VariableSim3Base {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static const VariableSim3Base::PerturbationSide PerturbationSide = PerturbationSide_;
    static const VariableSim3Base::PerturbationType PerturbationType = PerturbationType_;
    using BaseVariableType = VariableSim3_<PerturbationType_, PerturbationSide_>;

    virtual void applyPerturbation(const Vector7f& pert) override {
      this->_tainted = true;
      typedef VariableSim3Base BaseT;
      BaseT::EstimateType pert_m;
      switch (PerturbationType) {
        case Euler:
          pert_m = geometry3d::tas2s(pert);
          break;
        case Quaternion:
          pert_m = geometry3d::v2s(pert);
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

  using VariableSim3EulerRight = VariableSim3_<VariableSim3Base::Euler, VariableSim3Base::Right>;
  using VariableSim3EulerLeft  = VariableSim3_<VariableSim3Base::Euler, VariableSim3Base::Left>;
  using VariableSim3QuaternionRight =
    VariableSim3_<VariableSim3Base::Quaternion, VariableSim3Base::Right>;
  using VariableSim3QuaternionLeft =
    VariableSim3_<VariableSim3Base::Quaternion, VariableSim3Base::Left>;

} // namespace srrg2_solver
