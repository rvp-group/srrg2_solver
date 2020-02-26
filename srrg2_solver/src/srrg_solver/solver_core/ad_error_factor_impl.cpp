#pragma once
#include "ad_error_factor.h"
#include "ad_variable.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  template <typename ADErrorFactorType, int idx>
  struct JUpdater {
    static inline void update(ADErrorFactorType& adf) {
      JUpdater<ADErrorFactorType, idx - 1>::update(adf);
      adf.template _updateJacobianAD<idx>();
    }
  };

  template <typename ADErrorFactorType>
  struct JUpdater<ADErrorFactorType, 0> {
    static inline void update(ADErrorFactorType& adf) {
      adf.template _updateJacobianAD<0>();
    }
  };

  template <int ErrorDim_, typename... VariableTypes_>
  void ADErrorFactor_<ErrorDim_, VariableTypes_...>::errorAndJacobian(bool error_only) {
    Eigen::Matrix<DualValuef, ErrorDim, 1> ad_e = this->operator()(this->variables());
    convertMatrix(BaseType::_e, ad_e);
    if (error_only) {
      return;
    }
    JUpdater<ThisType, NumVariables - 1>::update(*this);
  }

  template <int ErrorDim_, typename... VariableTypes_>
  template <int idx>
  void ADErrorFactor_<ErrorDim_, VariableTypes_...>::_updateJacobianAD() {
    const int PertDim       = BaseType::template perturbationDim<idx>();
    using ADVarPerturbation = Eigen::Matrix<DualValuef, PertDim, 1>;
    ADVarPerturbation pert;
    auto& var(this->variables().template at<idx>());
    // var fixed, derivative is zero;
    if (var->status() != VariableBase::Active) {
      return;
    }
    pert.setZero();
    var->setEstimate(var->estimate());
    int c_base = BaseType::template perturbationOffset<idx>();
    for (int c = 0; c < PertDim; ++c) {
      // we reset the variable derivatives
      pert(c, 0).derivative = 1.f;
      var->applyPerturbationAD(pert);
      Eigen::Matrix<DualValuef, ErrorDim, 1> ad_e = this->operator()(this->variables());
      for (int r = 0; r < ErrorDim; ++r) {
        BaseType::_J(r, c + c_base) = ad_e(r).derivative;
      }
      var->setEstimate(var->estimate());
      pert(c, 0).derivative = 0.f;
    }
  }

} // namespace srrg2_solver
