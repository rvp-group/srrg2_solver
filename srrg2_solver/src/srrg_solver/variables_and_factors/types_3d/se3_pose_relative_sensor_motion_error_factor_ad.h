#pragma once
#include "srrg_solver/solver_core/ad_error_factor.h"
#include "variable_se3_ad.h"

namespace srrg2_solver {

  class SE3RelativeSensorMotionErrorFactorAD
    : public ADErrorFactor_<6, VariableSE3EulerRightAD, VariableSE3EulerRightAD>,
      public MeasurementOwnerEigen_<Isometry3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BaseType          = ADErrorFactor_<6, VariableSE3EulerRightAD, VariableSE3EulerRightAD>;
    using VariableTupleType = typename BaseType::VariableTupleType;
    using MeasurementType   = Isometry3f;

    BaseType::ADErrorVectorType operator()(VariableTupleType& vars) final {
      const Isometry3_<DualValuef>& sensor1_offset = vars.at<0>()->adEstimate();
      const Isometry3_<DualValuef>& sensor2_offset = vars.at<1>()->adEstimate();
      Isometry3_<DualValuef> pose = sensor1_offset.inverse() * sensor2_offset;
      Isometry3_<DualValuef> prediction = pose.inverse() * _ad_rel_motion * pose;
      Isometry3_<DualValuef> error = prediction.inverse() * _ad_measurement;
      return srrg2_core::geometry3d::t2v(error);
    }

    void setMeasurement(const Isometry3f& measurement_) override {
      _measurement = measurement_;
      convertMatrix(_ad_measurement, _measurement);
    }

    const Isometry3f& measurement() const override {
      return _measurement;
    }

    inline void setRelativeMotion(const Isometry3f& rel_motion_) {
      convertMatrix(_ad_rel_motion, rel_motion_);
    }

  protected:
    Isometry3_<DualValuef> _ad_measurement;
    Isometry3_<DualValuef> _ad_rel_motion;
  };
} // namespace srrg2_solver
