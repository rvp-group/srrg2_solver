#pragma once
#include "srrg_solver/solver_core/error_factor.h"
#include "srrg_solver/solver_core/factor_correspondence_driven.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3.h"
#include <srrg_geometry/geometry3d.h>
#include <srrg_pcl/point_types.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  /***** point to point *****/

  class SE3RectifiedStereoProjectiveErrorFactor
    : public ErrorFactor_<3, VariableSE3QuaternionRight> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType   = ErrorFactor_<3, VariableSE3QuaternionRight>;
    using FixedType  = Point4f;
    using MovingType = Point3f;

    inline void setFixed(const Point4f& fixed_) {
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

    inline void setBaselineLeftInRightPixels(const srrg2_core::Vector3f& baseline_pixels_) {
      _baseline_left_in_right_pixels = baseline_pixels_;
    }

    inline void setMeanDisparityPixels(const float& mean_disparity_pixels_) {
      _mean_disparity_pixels = mean_disparity_pixels_;
    }

  protected:
    const Vector3f* _moving_point                       = nullptr;
    const Vector4f* _fixed_point                        = nullptr;
    float _mean_disparity_pixels                        = 0;
    srrg2_core::Vector3f _baseline_left_in_right_pixels = srrg2_core::Vector3f::Zero();
    srrg2_core::Matrix3f _camera_matrix                 = srrg2_core::Matrix3f::Identity();
    srrg2_core::Vector2f _image_dim                     = srrg2_core::Vector2f::Zero();
  };

  // correspondence factor
  using SE3RectifiedStereoProjectiveErrorFactorCorrespondenceDriven =
    FactorCorrespondenceDriven_<SE3RectifiedStereoProjectiveErrorFactor,
                                Point4fVectorCloud,
                                Point3fVectorCloud>;

  class SE3RectifiedStereoProjectiveWithSensorErrorFactor
    : public SE3RectifiedStereoProjectiveErrorFactor {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType     = SE3RectifiedStereoProjectiveErrorFactor;
    using FixedType    = Point4f;
    using MovingType   = Point3f;
    using EstimateType = Isometry3f;

    void errorAndJacobian(bool error_only = false) final;

    //! @brief with sensor you refer to camera left
    inline void setSensorInRobot(const EstimateType& camera_left_in_robot_) {
      _robot_in_camera_left = camera_left_in_robot_.inverse();
    }

  protected:
    EstimateType _robot_in_camera_left = Isometry3f::Identity();
  };

  // correspondence factor
  using SE3RectifiedStereoProjectiveWithSensorErrorFactorCorrespondenceDriven =
    FactorCorrespondenceDriven_<SE3RectifiedStereoProjectiveWithSensorErrorFactor,
                                Point4fVectorCloud,
                                Point3fVectorCloud>;

} // namespace srrg2_solver
