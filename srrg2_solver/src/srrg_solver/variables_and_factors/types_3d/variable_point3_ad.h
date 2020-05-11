#pragma once
#include "srrg_solver/solver_core/ad_variable.h"
#include "variable_point3.h"

namespace srrg2_solver {

  using namespace srrg2_core;

  /** @brief 3D Point Variable with autodiff capabilities.
   */
  class VariablePoint3AD : public ADVariable_<VariablePoint3> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using VariableType             = VariablePoint3;
    using ADVariableType           = ADVariable_<VariableType>;
    using ADPerturbationVectorType = typename ADVariableType::ADPerturbationVectorType;
    using ADEstimateType           = typename ADVariableType::ADEstimateType;

    virtual ~VariablePoint3AD() = default;

    virtual void setZero() override {
      setEstimate(Vector3f::Zero());
    }

    virtual void applyPerturbationAD(const ADPerturbationVectorType& ad_pert) override {
      _ad_estimate += ad_pert;
    }
  };
} // namespace srrg2_solver
