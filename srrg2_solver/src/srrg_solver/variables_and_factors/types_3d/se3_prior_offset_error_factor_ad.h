#pragma once
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"
#include "srrg_solver/solver_core/ad_error_factor.h"
#include "srrg_solver/solver_core/measurement_owner.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  class SE3PriorOffsetErrorFactorAD
    : public ADErrorFactor_<6, VariableSE3QuaternionRightAD>,
      public MeasurementOwnerEigen_<Isometry3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType          = ADErrorFactor_<6, VariableSE3QuaternionRightAD>;
    using VariableTupleType = typename BaseType::VariableTupleType;
    using ADErrorVectorType = typename BaseType::ADErrorVectorType;

    ADErrorVectorType operator()(VariableTupleType& vars_) final {
      Isometry3_<DualValuef> prediction_ad =
        _inv_offset_ad * vars_.at<0>()->adEstimate() * _offset_ad;
      return geometry3d::t2v(_inv_measure_ad * prediction_ad);
    }

    void setMeasurement(const Isometry3f& measurement_) override {
      this->_measurement = measurement_;
      convertMatrix(_inv_measure_ad, measurement_.inverse());
    }

    void setOffset(const Isometry3f& offset_) {
      _offset = offset_;
      convertMatrix(_offset_ad, offset_);
      convertMatrix(_inv_offset_ad, offset_.inverse());
    }

  protected:
    Isometry3_<DualValuef> _inv_measure_ad = Isometry3_<DualValuef>::Identity();
    Isometry3_<DualValuef> _offset_ad      = Isometry3_<DualValuef>::Identity();
    Isometry3_<DualValuef> _inv_offset_ad  = Isometry3_<DualValuef>::Identity();
    Isometry3f _offset                     = Isometry3f::Identity();
  };
} // namespace srrg2_solver
