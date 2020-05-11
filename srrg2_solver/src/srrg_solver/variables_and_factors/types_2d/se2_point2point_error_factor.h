#pragma once
#include "srrg_solver/solver_core/ad_error_factor.h"
#include "srrg_solver/solver_core/factor_correspondence_driven_dynamic.h"
#include "variable_se2_ad.h"
#include <srrg_geometry/geometry2d.h>
#include <srrg_pcl/point_types.h>

namespace srrg2_solver {
  using namespace srrg2_core;
  /***** point to point *****/

  /** @brief ICP factor in 2D. This is a single point factor.
   *  The error is composed by:
   *  - e = difference between predicted and fixed points
   */
  class SE2Point2PointErrorFactor : public ErrorFactor_<2, VariableSE2Right> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using FixedType    = Point2f;
    using MovingType   = Point2f;
    using VariableType = VariableSE2Right;
    using EstimateType = VariableType::EstimateType;
    using BaseType     = ErrorFactor_<2, VariableType>;

    /**
     * @brief Provide a point form the fixed cloud to the optimization process
     * @param[in] Point2f from fixed cloud
     */
    inline void setFixed(const FixedType& fixed_) {
      fixed = &fixed_.coordinates();
    }
    /**
     * @brief Provide a point form the moving cloud to the optimization process
     * Used to compute the transformation that will bring the moving cloud on the fixed
     * @param[in] Point2f from moving cloud
     */
    inline void setMoving(const MovingType& moving_) {
      moving = &moving_.coordinates();
    }

    void errorAndJacobian(bool error_only = false) override;

  protected:
    const MovingType::VectorType* moving = nullptr;
    const FixedType::VectorType* fixed   = nullptr;
  };

  // correspondence factor
  using SE2Point2PointErrorFactorCorrespondenceDriven =
    FactorCorrespondenceDrivenDynamic_<SE2Point2PointErrorFactor>;

  /** @brief ICP factor in 2D exploiting pose of the sensor wrt the robot. This is a single
   * point factor.
   */
  class SE2Point2PointWithSensorErrorFactor : public SE2Point2PointErrorFactor {
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
  using SE2Point2PointWithSensorErrorFactorCorrespondenceDriven =
    FactorCorrespondenceDrivenDynamic_<SE2Point2PointWithSensorErrorFactor>;

} // namespace srrg2_solver
