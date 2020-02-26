#pragma once
#include "differential_drive_odom_predictor_ad.h"
#include "srrg_geometry/geometry2d.h"
#include "srrg_solver/variables_and_factors/types_2d/variable_se2_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h"
#include "srrg_solver/solver_core/ad_error_factor.h"

namespace srrg2_solver {

  class DifferentialDriveOdomSensor2DErrorFactorAD
    : public ADErrorFactor_<3, VariablePoint3AD, VariableSE2RightAD>,
      public DifferentialDriveOdomPredictorAD,
      public MeasurementOwnerEigen_<Isometry2f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BaseType             = ADErrorFactor_<3, VariablePoint3AD, VariableSE2RightAD>;
    using MeasurementOwnerType = MeasurementOwnerEigen_<Isometry2f>;
    using BasePredictorType    = DifferentialDriveOdomPredictorAD;
    using VariableTupleType    = typename BaseType::VariableTupleType;
    using ADErrorVectorType    = typename BaseType::ADErrorVectorType;

    ADErrorVectorType operator()(VariableTupleType& vars) final {
      //! to retrieve a variable you should know the position in the parameter pack
      //! and the type
      const Vector3_<DualValuef>& dd_params       = vars.at<0>()->adEstimate();
      const Isometry2_<DualValuef>& sensor_offset = vars.at<1>()->adEstimate();
      Isometry2_<DualValuef> robot_motion         = Isometry2_<DualValuef>::Identity();
      // do the computation (all in dual value)
      Vector3_<DualValuef> robot_motion_v = _predictRobotMotion(dd_params);
      robot_motion                        = srrg2_core::geometry2d::v2t(robot_motion_v);

      Isometry2_<DualValuef> sensor_motion_prediction =
        sensor_offset.inverse() * robot_motion * sensor_offset;
      Isometry2_<DualValuef> sensor_motion_error_isometry =
        _ad_measurement_inverse * sensor_motion_prediction;

      return geometry2d::t2v<DualValuef>(sensor_motion_error_isometry);
    }

    void setMeasurement(const Isometry2f& measurement_) override {
      _measurement = measurement_;
      Isometry2_<DualValuef> ad_measurement;
      convertMatrix(ad_measurement, _measurement);
      _ad_measurement_inverse = ad_measurement.inverse();
    }
    
    void serialize(ObjectData& odata, IdContext& context) override {
      BaseType::serialize(odata, context);
      ArrayData* mdata = new ArrayData;
      for (int i = 0; i < 2; ++i) {
        mdata->add(ticks()[i]);
      }
      odata.setField("ticks", mdata);
    }

    void deserialize(ObjectData& odata, IdContext& context) override {
      BaseType::deserialize(odata, context);
      Vector2f ticks;
      ArrayData* mdata = dynamic_cast<ArrayData*>(odata.getField("ticks"));
      for (int i = 0; i < 2; ++i) {
        ticks[i] = (*mdata)[i].getFloat();
      }
      setTicks(ticks);
    }

  protected:
    Isometry2_<DualValuef> _ad_measurement_inverse;
  };

} // namespace srrg2_solver
