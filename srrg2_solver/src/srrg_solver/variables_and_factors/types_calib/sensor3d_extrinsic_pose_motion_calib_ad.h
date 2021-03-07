#pragma once
#include "srrg_solver/solver_core/ad_error_factor.h"
#include "srrg_solver/solver_core/measurement_owner.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  /** @brief 3D factor to estimate sensor pose wrt to robot baseline when
   *  two indipendent tracking are available,
   *  i.e. sensor_in_world trajectory and robot_in_world trajectory
   *  carefull serialization of setters data, to be used for offline calibration
   */
  struct SE3SensorExtrinsicPoseMotionCalibAD
    : public ADErrorFactor_<6, VariableSE3QuaternionRightAD>,
      public MeasurementOwnerEigen_<Isometry3f> {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType             = ADErrorFactor_<6, VariableSE3QuaternionRightAD>;
    using VariableTupleType    = typename BaseType::VariableTupleType;
    using ADErrorVectorType    = typename BaseType::ADErrorVectorType;
    using TransformType        = Isometry3f;
    using TransformTypeAD      = Isometry3_<DualValuef>;
    using EstimateType         = VariableSE3QuaternionRightAD::EstimateType;
    using MeasurementOwnerType = MeasurementOwnerEigen_<EstimateType>;

    ADErrorVectorType operator()(VariableTupleType& vars) {
      const TransformTypeAD& offset_ad = vars.at<0>()->adEstimate();
      TransformTypeAD prediction_ad    = offset_ad.inverse() * _inv_from_ad * _to_ad * offset_ad;
      TransformTypeAD error_SE3        = _inv_measure_ad * prediction_ad;
      return geometry3d::t2v(error_SE3);
    }

    /**
     * @brief set sensor_in_sensor pose [t-1, t]
     * @param[in] TransformType (sensor_in_sensor pose)
     */
    void setMeasurement(const MeasurementType& measurement_) override {
      MeasurementOwnerType::setMeasurement(measurement_);
      convertMatrix(_inv_measure_ad, _measurement.inverse());
    }

    /**
     * @brief set robot in world [t-1]
     * @param[in] TransformType (robot_in_world pose)
     */
    void setFrom(const TransformType& from_) {
      _from = from_;
      convertMatrix(_inv_from_ad, from_.inverse());
    }

    /**
     * @brief set robot in world [t]
     * @param[in] TransformType (robot_in_world pose)
     */
    void setTo(const TransformType& to_) {
      _to = to_;
      convertMatrix(_to_ad, to_);
    }

    // ldg offline factor, serialization sucks
    void serialize(ObjectData& odata, IdContext& context) override;
    void deserialize(ObjectData& odata, IdContext& context) override;

    TransformType _from, _to;
    TransformTypeAD _inv_measure_ad = TransformTypeAD::Identity();
    TransformTypeAD _inv_from_ad    = TransformTypeAD::Identity();
    TransformTypeAD _to_ad          = TransformTypeAD::Identity();
  };
} // namespace srrg2_solver
