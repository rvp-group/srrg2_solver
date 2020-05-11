#pragma once
#include "differential_drive_odom_predictor_ad.h"
#include "srrg_geometry/geometry2d.h"
#include "srrg_solver/solver_core/ad_error_factor.h"
#include "srrg_solver/variables_and_factors/types_2d/variable_se2_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h"

namespace srrg2_solver {
  /**
   * @brief Intrinsic and 2D exstrinsic parameters error factor.
   * Compute the intrinsics parameters of a differential drive robot
   * [kl, kr, b] and the 2D pose of a sensor mounted on it given encoder's readings and the
   * kinematic model of the robot. The error is computed from the box-minus between the measured
   * sensor motion and the predicted one
   */
  class DifferentialDriveOdomSensor2DErrorFactorAD
    : public ADErrorFactor_<3, VariablePoint3AD, VariableSE2RightAD>,
      public DifferentialDriveOdomPredictorAD,
      public MeasurementOwnerEigen_<Isometry2f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BaseType             = ADErrorFactor_<3, VariablePoint3AD, VariableSE2RightAD>;
    using BasePredictorType    = DifferentialDriveOdomPredictorAD;
    using VariableTupleType    = typename BaseType::VariableTupleType;
    using ADErrorVectorType    = typename BaseType::ADErrorVectorType;
    using MeasurementOwnerType = MeasurementOwnerEigen_<Isometry2f>;
    using MeasurementTypeAD    = Isometry2_<DualValuef>;

    ADErrorVectorType operator()(VariableTupleType& vars) final {
      //! to retrieve a variable you should know the position in the parameter pack
      //! and the type
      const Vector3_<DualValuef>& dd_params       = vars.at<0>()->adEstimate();
      const Isometry2_<DualValuef>& sensor_offset = vars.at<1>()->adEstimate();
      // do the computation (all in dual value)
      Vector3_<DualValuef> robot_motion_v = _predictRobotMotion(dd_params);
      MeasurementTypeAD robot_motion      = srrg2_core::geometry2d::v2t(robot_motion_v);

      MeasurementTypeAD sensor_motion_prediction =
        sensor_offset.inverse() * robot_motion * sensor_offset;

      return geometry2d::t2v<DualValuef>(_ad_measurement_inverse * sensor_motion_prediction);
    }

    void setMeasurement(const Isometry2f& measurement_) override {
      MeasurementOwnerType::setMeasurement(measurement_);
      convertMatrix(_ad_measurement_inverse, _measurement.inverse());
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
    MeasurementTypeAD _ad_measurement_inverse =
      MeasurementTypeAD::Identity(); /**< Cache inverse autodiff measurement */
  };

} // namespace srrg2_solver
