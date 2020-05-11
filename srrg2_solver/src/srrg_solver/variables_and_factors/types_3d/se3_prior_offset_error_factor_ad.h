#pragma once
#include "srrg_solver/solver_core/ad_error_factor.h"
#include "srrg_solver/solver_core/measurement_owner.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  /** @brief 3D offset prior factor with autodiff.
   * Creates a unary factor to estimate the relative robot motion given
   * an offset using the relative motion wrt the offset
   */
  class SE3PriorOffsetErrorFactorAD : public ADErrorFactor_<6, VariableSE3QuaternionRightAD>,
                                      public MeasurementOwnerEigen_<Isometry3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType             = ADErrorFactor_<6, VariableSE3QuaternionRightAD>;
    using VariableTupleType    = typename BaseType::VariableTupleType;
    using ADErrorVectorType    = typename BaseType::ADErrorVectorType;
    using MeasurementOwnerType = MeasurementOwnerEigen_<Isometry3f>;

    ADErrorVectorType operator()(VariableTupleType& vars_) final;

    void setMeasurement(const MeasurementType& measurement_) override {
      MeasurementOwnerType::setMeasurement(measurement_);
      convertMatrix(_inv_measure_ad, _measurement.inverse());
    }

    /**
     * @brief Set the sensor pose wrt the robot base
     * @param[in] Isometry3f sensor pose in robot frame
     */
    void setSensorInRobot(const Isometry3f& sensor_in_robot_) {
      convertMatrix(_sensor_in_robot_ad, sensor_in_robot_);
      _robot_in_sensor_ad = _sensor_in_robot_ad.inverse();
    }

  protected:
    Isometry3_<DualValuef> _inv_measure_ad =
      Isometry3_<DualValuef>::Identity(); /**< Cache the inverse autodiff measurement */
    Isometry3_<DualValuef> _sensor_in_robot_ad =
      Isometry3_<DualValuef>::Identity(); /**< Cache the autodiff sensor offset */
    Isometry3_<DualValuef> _robot_in_sensor_ad =
      Isometry3_<DualValuef>::Identity(); /**< Cache the autodiff inverse sensor offset*/
  };
} // namespace srrg2_solver
