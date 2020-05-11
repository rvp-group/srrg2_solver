#include "se3_projective_depth_error_factor.h"
#include "srrg_solver/solver_core/ad_error_factor_impl.cpp"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {

  void SE3ProjectiveDepthErrorFactor::errorAndJacobian(bool error_only_) {
    assert(_image_dim.x() > 0);
    assert(_image_dim.y() > 0);
    assert(_camera_matrix.norm() > 0);
    _is_valid = false;

    // ds cache
    const Vector3f& point_in_moving = *_moving_point;
    const Vector3f& point_in_fixed  = *_fixed_point;
    assert(point_in_fixed.x() >= 0 && point_in_fixed.x() < _image_dim.x());
    assert(point_in_fixed.y() >= 0 && point_in_fixed.y() < _image_dim.y());
    assert(point_in_fixed.z() >= 0);
    const Isometry3f& X = _variables.at<0>()->estimate();

    // ds get point into current camera frame based on variable pose
    const Vector3f point_in_camera = X * point_in_moving;
    const float& depth             = point_in_camera.z();

    // ds skip invalid depths
    if (depth <= 0.0f) {
      return;
    }
    const float inverse_depth = 1.0f / depth;

    // ds compute points in image plane (reprojections), maintaining depth
    // ds TODO last row multiplication is redudant, remove
    const Vector3f point_in_image_homogeneous = _camera_matrix * point_in_camera;
    assert(_camera_matrix(2, 0) == 0.0f);
    assert(_camera_matrix(2, 1) == 0.0f);
    assert(_camera_matrix(2, 2) == 1.0f);
    const float& x_homogeneous = point_in_image_homogeneous.x();
    const float& y_homogeneous = point_in_image_homogeneous.y();
    const float x_in_image     = x_homogeneous * inverse_depth; // ds cols
    const float y_in_image     = y_homogeneous * inverse_depth; // ds rows

    // ds if the projected points are outside the image plane, skip the measurements
    if (x_in_image < 0 || x_in_image >= _image_dim.x() || y_in_image < 0 ||
        y_in_image >= _image_dim.y()) {
      return;
    }

    // ds compute error
    _e = Vector3f(x_in_image - point_in_fixed.x(), // ds error in image column reprojection
                  y_in_image - point_in_fixed.y(), // ds error in image row reprojection
                  depth - point_in_fixed.z());     // ds error in depth estimation

    // ds factor contributes
    _is_valid = true;

    // ds skip if desired
    if (error_only_) {
      return;
    }

    // ds compute the jacobian of the transformation
    Matrix3_6f jacobian_transform;

    // ds translation
    jacobian_transform.block<3, 3>(0, 0).setIdentity();

    // ds rotation (euler)
    jacobian_transform.block<3, 3>(0, 3) = -2 * geometry3d::skew(point_in_moving);

    // ds precompute
    const float inverse_depth_squared = inverse_depth * inverse_depth;

    // ds jacobian parts of the homogeneous division
    Matrix3f jacobian_homogeneous_division;
    // clang-format off
      jacobian_homogeneous_division <<
        inverse_depth, 0.0f, -x_homogeneous * inverse_depth_squared,
        0.0f, inverse_depth, -y_homogeneous * inverse_depth_squared,
        0.0f, 0.0f, 1.0f;
    // clang-format on

    // ds assemble final jacobian
    _J = jacobian_homogeneous_division * _camera_matrix * jacobian_transform;
  }

  INSTANTIATE(SE3ProjectiveDepthErrorFactor)
  INSTANTIATE(SE3ProjectiveDepthErrorFactorCorrespondenceDriven)

  void SE3ProjectiveDepthWithSensorErrorFactor::errorAndJacobian(bool error_only_) {
    assert(_image_dim.x() > 0);
    assert(_image_dim.y() > 0);
    assert(_camera_matrix.norm() > 0);
    _is_valid = false;

    // ds cache
    const Vector3f& point_in_moving = *_moving_point;
    const Vector3f& point_in_fixed  = *_fixed_point;
    assert(point_in_fixed.x() >= 0 && point_in_fixed.x() < _image_dim.x());
    assert(point_in_fixed.y() >= 0 && point_in_fixed.y() < _image_dim.y());
    assert(point_in_fixed.z() >= 0);
    const Isometry3f X = _variables.at<0>()->estimate();

    // ds get point into current camera frame based on variable pose
    const Vector3f point_in_robot  = X * point_in_moving;
    const Vector3f point_in_camera = _robot_in_sensor * point_in_robot;
    const float& depth             = point_in_camera.z();

    // ds skip invalid depths
    if (depth <= 0.0f) {
      return;
    }
    const float inverse_depth = 1.0f / depth;

    // ds compute points in image plane (reprojections), maintaining depth
    // ds TODO last row multiplication is redudant, remove
    const Vector3f point_in_image_homogeneous = _camera_matrix * point_in_camera;
    assert(_camera_matrix(2, 0) == 0.0f);
    assert(_camera_matrix(2, 1) == 0.0f);
    assert(_camera_matrix(2, 2) == 1.0f);
    const float& x_homogeneous = point_in_image_homogeneous.x();
    const float& y_homogeneous = point_in_image_homogeneous.y();
    const float x_in_image     = x_homogeneous * inverse_depth; // ds cols
    const float y_in_image     = y_homogeneous * inverse_depth; // ds rows

    // ds if the projected points are outside the image plane, skip the measurements
    if (x_in_image < 0 || x_in_image >= _image_dim.x() || y_in_image < 0 ||
        y_in_image >= _image_dim.y()) {
      return;
    }

    // ds compute error
    _e = Vector3f(x_in_image - point_in_fixed.x(), // ds error in image column reprojection
                  y_in_image - point_in_fixed.y(), // ds error in image row reprojection
                  depth - point_in_fixed.z());     // ds error in depth estimation

    // ds factor contributes
    _is_valid = true;

    // ds skip if desired
    if (error_only_) {
      return;
    }

    // ds compute the jacobian of the transformation
    Matrix3_6f jacobian_transform;

    // ds translation
    jacobian_transform.block<3, 3>(0, 0).setIdentity();

    // ds rotation (euler)
    jacobian_transform.block<3, 3>(0, 3) = -2 * geometry3d::skew(point_in_moving);

    // ds precompute
    const float inverse_depth_squared = inverse_depth * inverse_depth;

    // ds jacobian parts of the homogeneous division
    Matrix3f jacobian_homogeneous_division;
    // clang-format off
          jacobian_homogeneous_division <<
            inverse_depth, 0.0f, -x_homogeneous * inverse_depth_squared,
            0.0f, inverse_depth, -y_homogeneous * inverse_depth_squared,
            0.0f,          0.0f,                                   1.0f;
    // clang-format on

    Matrix3f map_in_sensor_R = (_robot_in_sensor * X).linear();
    // ds assemble final jacobian
    _J = jacobian_homogeneous_division * _camera_matrix * map_in_sensor_R * jacobian_transform;
  }

  INSTANTIATE(SE3ProjectiveDepthWithSensorErrorFactor)
  INSTANTIATE(SE3ProjectiveDepthWithSensorErrorFactorCorrespondenceDriven)

} // namespace srrg2_solver
