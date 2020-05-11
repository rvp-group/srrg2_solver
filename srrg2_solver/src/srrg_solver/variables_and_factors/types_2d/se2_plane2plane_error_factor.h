#pragma once
#include "srrg_solver/solver_core/ad_error_factor.h"
#include "srrg_solver/solver_core/factor_correspondence_driven_dynamic.h"
#include "variable_se2_ad.h"
#include <srrg_geometry/geometry2d.h>
#include <srrg_pcl/point_types.h>

namespace srrg2_solver {
  /** @brief ICP factor in 2D exploiting normals. This is a single point factor.
   *  The error is composed by:
   *  - e[0] = dot product between the fixed norm and the difference between predicted and fixed
   * points
   *  - e[1,2] = difference vector between predicted and fixed normals
   */
  class SE2Plane2PlaneErrorFactor : public ErrorFactor_<3, VariableSE2Right> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using FixedType    = PointNormal2f;
    using MovingType   = PointNormal2f;
    using VariableType = VariableSE2Right;
    using EstimateType = VariableType::EstimateType;
    using BaseType     = ErrorFactor_<3, VariableType>;

    /**
     * @brief Provide a point form the fixed cloud to the optimization process
     * @param[in] PointNormal2f from fixed cloud
     */
    inline void setFixed(const FixedType& fixed_) {
      _fixed_point  = &fixed_.coordinates();
      _fixed_normal = &fixed_.normal();
    }

    /**
     * @brief Provide a point form the moving cloud to the optimization process
     * Used to compute the transformation that will bring the moving cloud on the fixed
     * @param[in] PointNormal2f from moving cloud
     */
    inline void setMoving(const MovingType& moving_) {
      _moving_point  = &moving_.coordinates();
      _moving_normal = &moving_.normal();
    }

    void errorAndJacobian(bool error_only = false) override;

  protected:
    const MovingType::VectorType* _moving_point  = nullptr;
    const MovingType::VectorType* _moving_normal = nullptr;
    const FixedType::VectorType* _fixed_point    = nullptr;
    const FixedType::VectorType* _fixed_normal   = nullptr;
  };

  using SE2Plane2PlaneErrorFactorCorrespondenceDriven =
    FactorCorrespondenceDrivenDynamic_<SE2Plane2PlaneErrorFactor>;

  /** @brief ICP factor in 2D exploiting normals and pose of the sensor wrt the robot. This is a
   * single point factor.
   */
  class SE2Plane2PlaneWithSensorErrorFactor : public SE2Plane2PlaneErrorFactor {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void errorAndJacobian(bool error_only = false) final;

    /**
     * @brief Set the sensor pose wrt the robot base
     * @param[in] Isometry2f sensor pose in robot frame
     */
    inline void setSensorInRobot(const EstimateType& sensor_in_robot_) {
      _robot_in_sensor = sensor_in_robot_.inverse();
    }

  protected:
    EstimateType _robot_in_sensor = EstimateType::Identity(); /**< Calibrated pose of
                                                               * the sensor in robot frame
                                                               */
  };

  // correspondence factor
  using SE2Plane2PlaneWithSensorErrorFactorCorrespondenceDriven =
    FactorCorrespondenceDrivenDynamic_<SE2Plane2PlaneWithSensorErrorFactor>;

} // namespace srrg2_solver
