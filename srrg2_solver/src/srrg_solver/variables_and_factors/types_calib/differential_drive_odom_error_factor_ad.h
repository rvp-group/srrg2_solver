#pragma once
#include "differential_drive_odom_predictor_ad.h"
#include "srrg_geometry/geometry2d.h"
#include "srrg_solver/solver_core/ad_error_factor.h"
#include "srrg_solver/solver_core/measurement_owner.h"
#include "srrg_solver/variables_and_factors/types_2d/variable_se2_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h"

namespace srrg2_solver {

  /**
   * @brief DifferentialDrive parameters error factor.
   * Compute the intrinsics parameters of a differential drive robot
   * [kl, kr, b] given encoder's readings and the kinematic model of the robot.
   * The error is computed from the box-minus between the measured and predicted robot motion
   */
  class DifferentialDriveOdomErrorFactorAD : public ADErrorFactor_<3, VariablePoint3AD>,
                                             public MeasurementOwnerEigen_<Isometry2f>,
                                             public DifferentialDriveOdomPredictorAD {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType             = ADErrorFactor_<3, VariablePoint3AD>;
    using VariableTupleType    = typename BaseType::VariableTupleType;
    using BasePredictorType    = DifferentialDriveOdomPredictorAD;
    using ADErrorVectorType    = typename BaseType::ADErrorVectorType;
    using MeasurementOwnerType = MeasurementOwnerEigen_<Isometry2f>;
    using MeasurementTypeAD    = Isometry2_<DualValuef>;

    ADErrorVectorType operator()(VariableTupleType& dd_params) final {
      MeasurementTypeAD robot_motion = MeasurementTypeAD::Identity();
      // do the computation (all in dual value)
      ADErrorVectorType robot_motion_v =
        BasePredictorType::_predictRobotMotion(dd_params.at<0>()->adEstimate());
      robot_motion = srrg2_core::geometry2d::v2t(robot_motion_v);
      return geometry2d::t2v<DualValuef>(_ad_measurement_inverse * robot_motion);
    }

    inline void setMeasurement(const Isometry2f& measurement_) override {
      MeasurementOwnerType::setMeasurement(_measurement);
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
    MeasurementTypeAD _ad_measurement_inverse = MeasurementTypeAD::Identity();
  };
} // namespace srrg2_solver
