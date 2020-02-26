#pragma once
#include "srrg_solver/solver_core/ad_error_factor.h"
#include "srrg_solver/solver_core/factor_correspondence_driven.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"
#include <srrg_geometry/geometry3d.h>
#include <srrg_pcl/point_types.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  /***** point to point *****/

  class SE3ProjectiveDepthErrorFactor : public ErrorFactor_<3, VariableSE3QuaternionRight> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType   = ErrorFactor_<3, VariableSE3QuaternionRight>;
    using FixedType  = Point3f;
    using MovingType = Point3f;

    inline void setFixed(const Point3f& fixed_) {
      _fixed_point = &fixed_.coordinates();
    }
    inline void setMoving(const Point3f& moving_) {
      _moving_point = &moving_.coordinates();
    }

    void errorAndJacobian(bool error_only = false) override;

    inline void setCameraMatrix(const srrg2_core::Matrix3f& camera_matrix_) {
      _camera_matrix = camera_matrix_;
    }

    inline void setImageDim(const srrg2_core::Vector2f& image_dim_) {
      _image_dim = image_dim_;
    }

  protected:
    const Vector3f* _moving_point       = nullptr;
    const Vector3f* _fixed_point        = nullptr;
    srrg2_core::Matrix3f _camera_matrix = srrg2_core::Matrix3f::Identity();
    srrg2_core::Vector2f _image_dim     = srrg2_core::Vector2f::Zero();
  };

  // correspondence factor
  using SE3ProjectiveDepthErrorFactorCorrespondenceDriven =
    FactorCorrespondenceDriven_<SE3ProjectiveDepthErrorFactor,
                                Point3fVectorCloud,
                                Point3fVectorCloud>;

  class SE3ProjectiveDepthWithSensorErrorFactor : public SE3ProjectiveDepthErrorFactor {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType     = SE3ProjectiveDepthErrorFactor;
    using FixedType    = Point3f;
    using MovingType   = Point3f;
    using EstimateType = Isometry3f;

    void errorAndJacobian(bool error_only = false) final;

    inline void setSensorInRobot(const EstimateType& sensor_in_robot_) {
      _sensor_in_robot = sensor_in_robot_;
      _robot_in_sensor = _sensor_in_robot.inverse();
    }

  protected:
    EstimateType _sensor_in_robot = Isometry3f::Identity();
    EstimateType _robot_in_sensor = Isometry3f::Identity();
  };

  // correspondence factor
  using SE3ProjectiveDepthWithSensorErrorFactorCorrespondenceDriven =
    FactorCorrespondenceDriven_<SE3ProjectiveDepthWithSensorErrorFactor,
                                Point3fVectorCloud,
                                Point3fVectorCloud>;

  class SE3ProjectiveDepthWithSensorErrorFactorAD
    : public ADErrorFactor_<3, VariableSE3QuaternionRightAD> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType     = ADErrorFactor_<3, VariableSE3QuaternionRightAD>;
    using FixedType    = Point3f;
    using MovingType   = Point3f;
    using VariableType = VariableSE3QuaternionRightAD;

    using EstimateType   = VariableType::EstimateType;
    using ADEstimateType = VariableType::ADEstimateType;

    inline void setFixed(const Point3f& fixed_) {
      convertMatrix(_ad_fixed_point, fixed_.coordinates());
    }
    inline void setMoving(const Point3f& moving_) {
      convertMatrix(_ad_moving_point, moving_.coordinates());
    }

    inline void setCameraMatrix(const srrg2_core::Matrix3f& camera_matrix_) {
      convertMatrix(_ad_camera_matrix, camera_matrix_);
    }

    inline void setImageDim(const srrg2_core::Vector2f& image_dim_) {
      _image_dim = image_dim_;
    }

    inline void setSensorInRobot(const EstimateType& sensor_in_robot_) {
      convertMatrix(_ad_sensor_in_robot, sensor_in_robot_);
      _ad_robot_in_sensor = _ad_sensor_in_robot.inverse();
    }

    ADErrorVectorType operator()(VariableTupleType& vars) final;

  protected:
    Vector3_<DualValuef> _ad_moving_point;
    Vector3_<DualValuef> _ad_fixed_point;
    srrg2_core::Matrix3_<DualValuef> _ad_camera_matrix =
      srrg2_core::Matrix3_<DualValuef>::Identity();
    srrg2_core::Vector2f _image_dim    = srrg2_core::Vector2f::Zero();
    ADEstimateType _ad_sensor_in_robot = ADEstimateType::Identity();
    ADEstimateType _ad_robot_in_sensor = ADEstimateType::Identity();
  };

  // correspondence factor
  using SE3ProjectiveDepthWithSensorErrorFactorADCorrespondenceDriven =
    FactorCorrespondenceDriven_<SE3ProjectiveDepthWithSensorErrorFactorAD,
                                Point3fVectorCloud,
                                Point3fVectorCloud>;
} // namespace srrg2_solver
