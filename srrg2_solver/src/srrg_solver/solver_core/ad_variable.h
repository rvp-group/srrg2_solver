#pragma once
#include "variable.h"

namespace srrg2_solver {

  using namespace srrg2_core;

  /*! @brief Interface for a variable that supports autodiff, which is defined through a non-AD
     variable type (see variable.h). This class gives the possibile to use auto diff in cases where
     the estimate type is a non-Eigen object. This is done by defining:
      - dual2float()
      - float2dual()
      Which specify how to convert the estimate type from floats to dual values

      Further to define your AD variable you must override the applyPerturbationAD(), which specify
     how to apply the perturbation to the estimate.
     The EstimateType (and so ADEstimateType) must implement a static method  Identity()
     that sets the estimate to the origin of the its domain.
  */
  template <typename BaseVariableType_>
  class ADVariableBase_ : public BaseVariableType_ {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseVariableType           = BaseVariableType_;
    static const int PerturbationDim = BaseVariableType::PerturbationDim; /*!< Total
                                                                            perturbation
                                                                            dimension*/
    using EstimateType = typename BaseVariableType::EstimateType;         /*!< Extract
                                                                            estimate type
                                                                            from base variable */

    using PerturbationVectorType =
      typename Eigen::Matrix<float, PerturbationDim, 1>; /*!< Perturbation
                                                           vector
                                                           type */

    using ADEstimateType = typename BaseVariableType::ADEstimateType; /*!< Extract AD estimate
                                                                        type from base variable */
    using ADPerturbationVectorType =
      typename Eigen::Matrix<DualValuef, PerturbationDim, 1>; /*!< AD
                                                                perturbation
                                                                vector
                                                                type */
    /*! Set the value of _estimate and _ad_estimate, which is obtained through the float2dual()
      function
      @param[in] est value of the estimate
    */
    virtual void setEstimate(const EstimateType& est) override {
      BaseVariableType::_estimate = est;
      this->_tainted              = true;
      float2dual();
    }
    /*! Push the value of the variable in the stack */
    virtual void push() {
      this->_tainted = true;
      BaseVariableType::_stack.push_front(BaseVariableType::_estimate);
    }
    /*! Pop the value of the variable in the stack */
    virtual void pop() {
      this->_tainted = true;
      assert(!BaseVariableType::_stack.empty());
      setEstimate(BaseVariableType::_stack.front());
      BaseVariableType::_stack.pop_front();
    }

    /*! Copies the values from _ad_estimate (autodiff) to _estimate (non autodiff) */
    virtual void dual2float() = 0;

    /*! Copies the values from _estimate (non autodiff) to _ad_estimate (autodiff) */
    virtual void float2dual() = 0;

    /*! @return The estimate in dual values */
    inline const ADEstimateType& adEstimate() const {
      return _ad_estimate;
    }

    /*! Apply the perturbation to the AD estimate and then call dual2float() to
      get the new estimate in non-AD format
      @param[in] pert perturbation vector in floats
    */
    virtual void applyPerturbation(const PerturbationVectorType& pert) override {
      this->_tainted = true;
      ADPerturbationVectorType ad_pert;
      convertMatrix(ad_pert, pert);
      applyPerturbationAD(ad_pert);
      dual2float();
    }

    /*! Apply the perturbation in DualValuef to the AD estimate - must be overrided in the derived
      class
      @param[in] pert perturbation vector in DualValuef
     */
    virtual void applyPerturbationAD(const ADPerturbationVectorType& pert) = 0;

  protected:
    ADEstimateType _ad_estimate = ADEstimateType::Identity(); /*!< Estimate
                                                                in
                                                                DualValuef (AD) -
                                                                ADEstimateType must implement
                                                                a static method  Identity()
                                                               that sets the estimate to
                                                               the origin of the domain*/
  };

  /*! @brief Autodiff variable for Eigen typed estimates. The dual2float() and float2dual() comes
    for free. In this case only applyPerturbationAD() must be overrided in the derived class */
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
