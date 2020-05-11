#pragma once
#include "srrg_solver/solver_core/ad_error_factor.h"
#include "srrg_solver/solver_core/factor_correspondence_driven_dynamic.h"
#include "variable_se3_ad.h"
#include <srrg_pcl/point_types.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  /** @brief ICP factor in 3D. This is a single point factor.
   *  The error is composed by:
   *  - e = difference between predicted and fixed points
   */
  class SE3Point2PointErrorFactor : public ErrorFactor_<3, VariableSE3QuaternionRight> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using FixedType    = Point3f;
    using MovingType   = Point3f;
    using VariableType = VariableSE3QuaternionRight;
    using EstimateType = VariableType::EstimateType;
    using BaseType     = ErrorFactor_<3, VariableType>;

    /**
     * @brief Provide a point form the fixed cloud to the optimization process
     * @param[in] Point3f from fixed cloud
     */
    inline void setFixed(const Point3f& fixed_) {
      fixed = &fixed_.coordinates();
    }
    /**
     * @brief Provide a point form the moving cloud to the optimization process
     * Used to compute the transformation that will bring the moving cloud on the fixed
     * @param[in] Point3f from moving cloud
     */
    inline void setMoving(const Point3f& moving_) {
      moving = &moving_.coordinates();
    }

    void errorAndJacobian(bool error_only = false) override;

  protected:
    const Isometry3f* X    = nullptr;
    const Vector3f* moving = nullptr;
    const Vector3f* fixed  = nullptr;
  };

  using SE3Point2PointErrorFactorCorrespondenceDriven =
    FactorCorrespondenceDrivenDynamic_<SE3Point2PointErrorFactor>;

  /** @brief ICP factor in 3D exploiting pose of the sensor wrt the robot. This is a single
   * point factor.
   */
  class SE3Point2PointWithSensorErrorFactor : public SE3Point2PointErrorFactor {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void errorAndJacobian(bool error_only = false) final;

    /**
     * @brief Set the sensor pose wrt the robot base
     * @param[in] Isometry3f sensor pose in robot frame
     */
    inline void setSensorInRobot(const EstimateType& sensor_in_robot_) {
      _robot_in_sensor = sensor_in_robot_.inverse();
    }

  protected:
    EstimateType _robot_in_sensor = EstimateType::Identity(); /**< Calibrated pose of
                                                               * the sensor in robot frame
                                                               */
  };

  using SE3Point2PointWithSensorErrorFactorCorrespondenceDriven =
    FactorCorrespondenceDrivenDynamic_<SE3Point2PointWithSensorErrorFactor>;
} // namespace srrg2_solver
