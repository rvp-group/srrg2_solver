#pragma once
#include "variable.h"

namespace srrg2_solver {

  using namespace srrg2_core;

  // variable that supports autodiff
  // declare it as usual
  // ADVariable_<Isometry3_, 6>
  // the estimate should support template scalar parameters
  // override just one method: the applyPerturbationAD(ADPerturbationVectorType)

  template <typename BaseVariableType_>
  class ADVariableBase_ : public BaseVariableType_ {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //! handy base type
    using BaseVariableType = BaseVariableType_;
    // dimension of the perturbation
    static const int PerturbationDim = BaseVariableType::PerturbationDim;
    using EstimateType               = typename BaseVariableType::EstimateType;

    // plain perturbation vector
    using PerturbationVectorType = typename Eigen::Matrix<float, PerturbationDim, 1>;

    // the estimate in dual value
    using ADEstimateType = typename BaseVariableType::ADEstimateType;
    // the perturbation vector in dualvalue
    using ADPerturbationVectorType = typename Eigen::Matrix<DualValuef, PerturbationDim, 1>;

    // setEstimate updates the jacobian of the perturbation
    virtual void setEstimate(const EstimateType& est) override {
      BaseVariableType::_estimate = est;
      this->_tainted              = true;
      convertMatrix(_ad_estimate, BaseVariableType::_estimate);
    }

    virtual void push() {
      this->_tainted = true;
      BaseVariableType::_stack.push_front(BaseVariableType::_estimate);
    }

    virtual void pop() {
      this->_tainted = true;
      assert(!BaseVariableType::_stack.empty());
      setEstimate(BaseVariableType::_stack.front());
      BaseVariableType::_stack.pop_front();
    }

    // copies the values from _ad_estimate (autodiff) to _estimate (non autodiff)
    virtual void dual2float() = 0;

    // copies the values from _estimate (non autodiff) to _ad_estimate (autodiff)
    virtual void float2dual() = 0;

    // returns the estimate in autodiff values
    inline const ADEstimateType& adEstimate() const {
      return _ad_estimate;
    }

    // applyPerturbation recomputes the jacobian
    virtual void applyPerturbation(const PerturbationVectorType& pert) override {
      this->_tainted = true;
      ADPerturbationVectorType ad_pert;
      convertMatrix(ad_pert, pert);
      applyPerturbationAD(ad_pert);
      dual2float();
    }

    //! overrideme by applying the perturbation to the inner _ad_estimate variable
    //! the perturbation vector is in dualvaluef
    virtual void applyPerturbationAD(const ADPerturbationVectorType& pert) = 0;

  protected:
    ADEstimateType _ad_estimate = ADEstimateType::Identity();
  };

  template <typename BaseVariableType_>
  class ADVariable_ : public ADVariableBase_<BaseVariableType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ADVariableType   = ADVariableBase_<BaseVariableType_>;
    using BaseVariableType = BaseVariableType_;

    virtual void float2dual() override {
      convertMatrix(ADVariableType::_ad_estimate, BaseVariableType::_estimate);
    }

    virtual void dual2float() override {
      convertMatrix(BaseVariableType::_estimate, ADVariableType::_ad_estimate);
    }
  };

} // namespace srrg2_solver
