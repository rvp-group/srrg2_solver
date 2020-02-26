#include "variable_se2_ad.h"
#include "srrg_solver/solver_core/instance_macros.h"
#include "srrg_solver/solver_core/variable_impl.cpp"

namespace srrg2_solver {
  using namespace srrg2_core;

  template <typename VariableSE2_>
  VariableSE2AD_<VariableSE2_>::~VariableSE2AD_(){}

  template <typename VariableSE2_>
  void VariableSE2AD_<VariableSE2_>::applyPerturbationAD(const ADPerturbationVectorType& pert) {
    ADEstimateType pert_m;
    pert_m = geometry2d::v2t(pert);

    switch (VariableType::PerturbationSide) {
    case VariableSE2Base::Left:
      ADVariableType::_ad_estimate = pert_m * ADVariableType::_ad_estimate;
      break;
    case VariableSE2Base::Right:
      ADVariableType::_ad_estimate = ADVariableType::_ad_estimate * pert_m;
      break;
    default:
      assert(0);
    }
  }

  INSTANTIATE(VariableSE2RightAD)
  INSTANTIATE(VariableSE2LeftAD)
  
} // namespace srrg2_solver
