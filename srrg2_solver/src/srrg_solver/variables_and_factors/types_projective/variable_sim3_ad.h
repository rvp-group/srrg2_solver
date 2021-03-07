#pragma once
#include "srrg_solver/solver_core/ad_variable.h"
#include "variable_sim3.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  // to declare an ad variable extend and specialize the advariable class
  // the template argument is a variable  (non autodiff)

  template <typename VariableSim3_>
  class VariableSim3AD_ : public ADVariable_<VariableSim3_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*always replicate these typedefs. It's annoying but current compilers aren't smart enough.*/
    typedef ADVariable_<VariableSim3_> ADVariableType;
    typedef VariableSim3_ VariableType;
    typedef typename ADVariableType::ADPerturbationVectorType ADPerturbationVectorType;
    typedef typename ADVariableType::ADEstimateType ADEstimateType;

    inline void applyPerturbationAD(const ADPerturbationVectorType& pert) override {
      ADEstimateType pert_m;
      switch (VariableType::PerturbationType) {
        case VariableSim3Base::Euler:
          pert_m = geometry3d::tas2s(pert);
          break;
        case VariableSim3Base::Quaternion:
          pert_m = geometry3d::v2s(pert);
      }

      switch (VariableType::PerturbationSide) {
        case VariableSim3Base::Left:
          ADVariableType::_ad_estimate = pert_m * ADVariableType::_ad_estimate;
          break;
        case VariableSim3Base::Right:
          ADVariableType::_ad_estimate = ADVariableType::_ad_estimate * pert_m;
          break;
        default:
          assert(0);
      }
    }
  };

  typedef VariableSim3AD_<VariableSim3EulerRight> VariableSim3EulerRightAD;
  typedef VariableSim3AD_<VariableSim3EulerLeft> VariableSim3EulerLeftAD;
  typedef VariableSim3AD_<VariableSim3QuaternionRight> VariableSim3QuaternionRightAD;
  typedef VariableSim3AD_<VariableSim3QuaternionLeft> VariableSim3QuaternionLeftAD;
} // namespace srrg2_solver
