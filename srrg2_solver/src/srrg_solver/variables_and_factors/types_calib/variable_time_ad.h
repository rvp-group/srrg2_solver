#pragma once
#include "variable_time.h"
#include <srrg_solver/solver_core/ad_variable.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  /**
   * @brief 1D Time autodiff Variable
   */

  class VariableTimeAD : public ADVariable_<VariableTime> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*always replicate these typedefs. It's annoying but current compilers aren't smart enough.*/
    using VariableType             = VariableTime;
    using ADVariableType           = ADVariable_<VariableType>;
    using ADPerturbationVectorType = typename ADVariableType::ADPerturbationVectorType;
    using ADEstimateType           = typename ADVariableType::ADEstimateType;

    virtual ~VariableTimeAD();

    virtual void applyPerturbationAD(const ADPerturbationVectorType& pert) override {
      ADVariableType::_ad_estimate += pert;
    }
  };

} // namespace srrg2_solver
