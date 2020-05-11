#pragma once
#include "srrg_solver/solver_core/ad_error_factor.h"
#include "srrg_solver/solver_core/factor_correspondence_driven_dynamic.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"

namespace srrg2_solver {

  /**
   * @brief Offset from motion Factor.
   * Estimate the offset of a body rigidly attached to the robot frame given the relative motion of
   * the robot and the relative motion of the body.
   */
  class SE3PoseMotionErrorFactorAD : public ADErrorFactor_<6, VariableSE3QuaternionRightAD>,
                                     public MeasurementOwnerEigen_<Isometry3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType          = ADErrorFactor_<6, VariableSE3QuaternionRightAD>;
    using VariableTupleType = typename BaseType::VariableTupleType;
    using FixedType         = Isometry3f;
    using MovingType        = Isometry3f;
    using MeasurementTypeAD = Isometry3_<DualValuef>;
    using MovingTypeAD      = Isometry3_<DualValuef>;

    BaseType::ADErrorVectorType operator()(VariableTupleType& vars) final {
      Isometry3_<DualValuef> pose_      = vars.at<0>()->adEstimate();
      Isometry3_<DualValuef> prediction = pose_.inverse() * _ad_rel_motion * pose_;
      Isometry3_<DualValuef> error      = prediction.inverse() * _ad_measurement;
      return srrg2_core::geometry3d::t2v(error);
    }

    inline void setMeasurement(const FixedType& measurement_) override {
      _measurement = measurement_;
      convertMatrix(_ad_measurement, _measurement);
    }

    /**
     * @brief set the robot motion to estimate the sensor pose
     * @param[in] Isometry3f robot motion
     */
    inline void setRelativeMotion(const MovingType& rel_motion_) {
      convertMatrix(_ad_rel_motion, rel_motion_);
    }

    /**
     * @brief fulfill interface for FactorCorrespondenceDrivenDynamic
     *        calls setMeasurement
     * @param[in] Isometry3f measurement (sensor-sensor pose)
     */
    inline void setFixed(const FixedType& measurement_) {
      setMeasurement(measurement_);
    }

    /**
     * @brief fulfill interface for FactorCorrespondenceDrivenDynamic
     *        calls setRelativeMotion
     * @param[in] Isometry3f robot motion
     */
    inline void setMoving(const MovingType& rel_motion_) {
      setRelativeMotion(rel_motion_);
    }

  protected:
    MeasurementTypeAD _ad_measurement = MeasurementTypeAD::Identity();
    MovingTypeAD _ad_rel_motion       = MovingTypeAD::Identity();
  };

  using SE3PoseMotionErrorFactorDataDriven =
    FactorCorrespondenceDrivenDynamic_<SE3PoseMotionErrorFactorAD>;
} // namespace srrg2_solver
