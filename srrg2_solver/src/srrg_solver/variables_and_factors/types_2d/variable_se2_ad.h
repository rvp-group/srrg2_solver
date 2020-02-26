#pragma once
#include "srrg_solver/solver_core/ad_variable.h"
#include "variable_se2.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  // to declate an ad variable extend and specialize the advariable class
  // the template argument is a variable  (non autodiff)
  template <typename VarableSE2_>
  class VariableSE2AD_ : public ADVariable_<VarableSE2_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*always replicate these typedefs. It's annoying but current compilers aren't smart enough.*/
    typedef ADVariable_<VarableSE2_> ADVariableType;
    typedef VarableSE2_ VariableType;
    typedef typename ADVariableType::ADPerturbationVectorType ADPerturbationVectorType;
    typedef typename ADVariableType::ADEstimateType ADEstimateType;

    virtual ~VariableSE2AD_();
    virtual void applyPerturbationAD(const ADPerturbationVectorType& pert);
  };

  using VariableSE2RightAD = VariableSE2AD_<VariableSE2Right>;
  using VariableSE2LeftAD  = VariableSE2AD_<VariableSE2Left>;
} // namespace srrg2_solver
