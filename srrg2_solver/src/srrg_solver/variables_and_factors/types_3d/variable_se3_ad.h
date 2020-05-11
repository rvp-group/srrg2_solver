#pragma once
#include "srrg_solver/solver_core/ad_variable.h"
#include "variable_se3.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  /** @brief SE3 Pose AD Variable. The template argument is the
   * base Variable class
   */

  template <typename VarableSE3_>
  class VariableSE3AD_ : public ADVariable_<VarableSE3_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*always replicate these typedefs. It's annoying but current compilers aren't smart enough.*/
    typedef ADVariable_<VarableSE3_> ADVariableType;
    typedef VarableSE3_ VariableType;
    typedef typename ADVariableType::ADPerturbationVectorType ADPerturbationVectorType;
    typedef typename ADVariableType::ADEstimateType ADEstimateType;

    virtual void applyPerturbationAD(const ADPerturbationVectorType& pert) {
      ADEstimateType pert_m;
      switch (VariableType::PerturbationType) {
        case VariableSE3Base::Euler:
          pert_m = geometry3d::ta2t(pert);
          break;
        case VariableSE3Base::Quaternion:
          pert_m = geometry3d::v2t(pert);
      }

      switch (VariableType::PerturbationSide) {
        case VariableSE3Base::Left:
          ADVariableType::_ad_estimate = pert_m * ADVariableType::_ad_estimate;
          break;
        case VariableSE3Base::Right:
          ADVariableType::_ad_estimate = ADVariableType::_ad_estimate * pert_m;
          break;
        default:
          assert(0);
      }
    }
  };

  typedef VariableSE3AD_<VariableSE3EulerRight> VariableSE3EulerRightAD;
  typedef VariableSE3AD_<VariableSE3EulerLeft> VariableSE3EulerLeftAD;
  typedef VariableSE3AD_<VariableSE3QuaternionRight> VariableSE3QuaternionRightAD;
  typedef VariableSE3AD_<VariableSE3QuaternionLeft> VariableSE3QuaternionLeftAD;
} // namespace srrg2_solver
