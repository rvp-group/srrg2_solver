#pragma once
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"
#include "srrg_solver/solver_core/ad_error_factor.h"
#include "srrg_solver/solver_core/factor_correspondence_driven_dynamic.h"

namespace srrg2_solver {
  class SE3PoseMotionErrorFactorAD : public ADErrorFactor_<6, VariableSE3QuaternionRightAD>,
                                     public MeasurementOwnerEigen_<Isometry3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = ADErrorFactor_<6, VariableSE3QuaternionRightAD>;
    using VariableTupleType = typename BaseType::VariableTupleType;
    using FixedType  = Isometry3f;
    using MovingType = Isometry3f;

    BaseType::ADErrorVectorType operator()(VariableTupleType& vars) final {
      Isometry3_<DualValuef> pose_ = vars.at<0>()->adEstimate();
      Isometry3_<DualValuef> prediction = pose_.inverse() * _ad_rel_motion * pose_;
      Isometry3_<DualValuef> error      = prediction.inverse() * _ad_measurement;
      return srrg2_core::geometry3d::t2v(error);
    }

    inline void setMeasurement(const Isometry3f& measurement_) override {
      _measurement = measurement_;
      convertMatrix(_ad_measurement, _measurement);
    }

    inline void setRelativeMotion(const Isometry3f& rel_motion_) {
      convertMatrix(_ad_rel_motion, rel_motion_);
    }

    inline void setFixed(const Isometry3f& measurement_) {
      setMeasurement(measurement_);
    }

    inline void setMoving(const Isometry3f& rel_motion_) {
      setRelativeMotion(rel_motion_);
    }

  protected:
    Isometry3_<DualValuef> _ad_measurement;
    Isometry3f _measurement = Isometry3f::Identity();
    Isometry3_<DualValuef> _ad_rel_motion;
  };

  using SE3PoseMotionErrorFactorDataDriven =
    FactorCorrespondenceDrivenDynamic_<SE3PoseMotionErrorFactorAD>;
} // namespace srrg2_solver
