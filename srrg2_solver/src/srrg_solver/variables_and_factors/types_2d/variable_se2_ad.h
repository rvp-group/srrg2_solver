#pragma once
#include "srrg_solver/solver_core/ad_variable.h"
#include "variable_se2.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  /** @brief SE2 Pose AD Variable.
   * The template argument is the base Variable class
   */
  template <typename VarableSE2_>
  class VariableSE2AD_ : public ADVariable_<VarableSE2_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*always replicate these typedefs. It's annoying but current compilers aren't smart enough.*/
    using VariableType             = VarableSE2_;
    using ADVariableType           = ADVariable_<VariableType>;
    using ADPerturbationVectorType = typename ADVariableType::ADPerturbationVectorType;
    using ADEstimateType           = typename ADVariableType::ADEstimateType;

    virtual ~VariableSE2AD_();
    virtual void applyPerturbationAD(const ADPerturbationVectorType& pert);
  };

  using VariableSE2RightAD = VariableSE2AD_<VariableSE2Right>;
  using VariableSE2LeftAD  = VariableSE2AD_<VariableSE2Left>;
} // namespace srrg2_solver
