#pragma once
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"
#include "srrg_solver/solver_core/ad_error_factor.h"
#include "srrg_solver/solver_core/measurement_owner.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  class SE3PriorErrorFactorAD : public ADErrorFactor_<6, VariableSE3QuaternionRightAD>,
                                          public MeasurementOwnerEigen_<Isometry3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType          = ADErrorFactor_<6, VariableSE3QuaternionRightAD>;
    using VariableTupleType = typename BaseType::VariableTupleType;
    using ADErrorVectorType = typename BaseType::ADErrorVectorType;

    ADErrorVectorType operator()(VariableTupleType& vars_) final {
      const Isometry3_<DualValuef>& T  = vars_.at<0>()->adEstimate();
      Isometry3_<DualValuef> error_SE3 = _inv_measure * T;
      return geometry3d::t2v(error_SE3);
    }

    void setMeasurement(const Isometry3f& measurement_) override {
      _measurement        = measurement_;
      Isometry3f inv_meas = measurement_.inverse();
      convertMatrix(_inv_measure, inv_meas);
    }

  protected:
    Isometry3_<DualValuef> _inv_measure = Isometry3_<DualValuef>::Identity();
  };

} // namespace srrg2_solver
