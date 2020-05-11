#pragma once
#include "srrg_solver/solver_core/ad_error_factor.h"
#include "srrg_solver/solver_core/factor_correspondence_driven_dynamic.h"
#include "variable_se3_ad.h"
#include <srrg_pcl/point_types.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  /** @brief ICP factor in 3D exploiting normals. This is a single point factor.
   *  The error is composed by:
   *  - e[0] = dot product between the fixed norm and the difference between predicted and fixed
   * points
   *  - e[1,2,3] = difference vector between predicted and fixed normals
   */
  class SE3Plane2PlaneErrorFactor : public ErrorFactor_<4, VariableSE3QuaternionRight> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using FixedType    = PointNormal3f;
    using MovingType   = PointNormal3f;
    using VariableType = VariableSE3QuaternionRight;
    using EstimateType = VariableType::EstimateType;
    using BaseType     = ErrorFactor_<4, VariableType>;

    /**
     * @brief set point from the fixed cloud
     * @param point in the fixed cloud
     */
    inline void setFixed(const PointNormal3f& fixed_) {
      fixed_point  = &fixed_.coordinates();
      fixed_normal = &fixed_.normal();
    }

    /**
     * @brief set point from the moving cloud
     * @param point in the moving cloud corresponding to the fixed
     */
    inline void setMoving(const PointNormal3f& moving_) {
      moving_point  = &moving_.coordinates();
      moving_normal = &moving_.normal();
    }

    void errorAndJacobian(bool error_only) override;

  protected:
    const Vector3f* moving_point  = nullptr;
    const Vector3f* fixed_point   = nullptr;
    const Vector3f* moving_normal = nullptr;
    const Vector3f* fixed_normal  = nullptr;
  };

  using SE3Plane2PlaneErrorFactorCorrespondenceDriven =
    FactorCorrespondenceDrivenDynamic_<SE3Plane2PlaneErrorFactor>;

  /** @brief ICP factor in 3D exploiting normals and pose of the sensor wrt the robot. This is a
   * single point factor.
   */
  class SE3Plane2PlaneWithSensorErrorFactor : public SE3Plane2PlaneErrorFactor {
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

  using SE3Plane2PlaneWithSensorErrorFactorCorrespondenceDriven =
    FactorCorrespondenceDrivenDynamic_<SE3Plane2PlaneWithSensorErrorFactor>;

} // namespace srrg2_solver
