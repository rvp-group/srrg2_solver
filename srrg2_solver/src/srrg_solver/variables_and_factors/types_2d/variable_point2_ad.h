#pragma once
#include "srrg_solver/solver_core/ad_variable.h"
#include "variable_point2.h"

namespace srrg2_solver {

  using namespace srrg2_core;

  /** @brief 2D Point Variable with autodiff capabilities.
   */
  class VariablePoint2AD : public ADVariable_<VariablePoint2> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using VariableType             = VariablePoint2;
    using ADVariableType           = ADVariable_<VariableType>;
    using ADPerturbationVectorType = typename ADVariableType::ADPerturbationVectorType;
    using ADEstimateType           = typename ADVariableType::ADEstimateType;

    virtual ~VariablePoint2AD() = default;

    virtual void setZero() override {
      setEstimate(Vector2f::Zero());
    }

    virtual void applyPerturbationAD(const ADPerturbationVectorType& ad_pert) override {
      _ad_estimate += ad_pert;
    }
  };

} // namespace srrg2_solver
