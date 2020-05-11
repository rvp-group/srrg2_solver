#include "se3_rectified_stereo_projective_error_factor.h"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  //! error function and factor for plain 3D stereo point to plane measurements
  inline void SE3RectifiedStereoProjectiveErrorFactor::errorAndJacobian(bool error_only) {
    assert(_image_dim.x() > 0);
    assert(_image_dim.y() > 0);
    assert(_camera_matrix.norm() > 0);
    assert(_baseline_left_in_right_pixels.norm() > 0);
    _is_valid              = false;
    const Vector3f& moving = *(this->_moving_point);
    const Vector4f& fixed  = *(this->_fixed_point);
    const Isometry3f& X    = _variables.at<0>()->estimate();
    assert(!moving.hasNaN());
    assert(!fixed.hasNaN());
    assert(!X.matrix().hasNaN());

    // ds cache disparity
    const float disparity_pixels = fixed(0) - fixed(2);
    assert(disparity_pixels >= 0);

    // ds get point into current camera frame based on variable pose
    const Vector3f point_in_camera_left = X * moving;

    // ds drop points with invalid depth
    if (point_in_camera_left.z() <= 0) {
      return;
    }

    // ds compute points in image plane (reprojections) - assuming rigid stereo!
    const Vector3f abc_left = _camera_matrix * point_in_camera_left;
    assert(!abc_left.hasNaN());
    const Vector3f abc_right            = abc_left + _baseline_left_in_right_pixels;
    const float& c_left                 = abc_left.z();
    const float& c_right                = abc_right.z();
    const Vector3f point_in_image_left  = abc_left / c_left;
    const Vector3f point_in_image_right = abc_right / c_right;

    // ds if the projected points are outside the image plane, skip the measurements
    if (point_in_image_left.x() < 0 || point_in_image_left.x() > _image_dim.x() ||
        point_in_image_left.y() < 0 || point_in_image_left.y() > _image_dim.y() ||
        point_in_image_right.x() < 0 || point_in_image_right.x() > _image_dim.x() ||
        point_in_image_right.y() < 0 || point_in_image_right.y() > _image_dim.y()) {
      return;
    }
    // assert(fixed(1) == fixed(3) && "images are not rectified (or correspondences invalid)");

    // ds compute error (we compute the vertical error only once, since we assume rectified cameras)
    // ds MUST BE COMPENSATED WITH DOUBLE Y WEIGHT IN OMEGA for consistency with regular stereo
    _e = Vector3f(point_in_image_left.x() - fixed(0),
                  point_in_image_left.y() - fixed(1),
                  point_in_image_right.x() - fixed(2));

    // ds factor contributes at this point
    _is_valid = true;

    // ds skip if desired
    if (error_only) {
      return;
    }

    // ds compute the jacobian of the transformation - with optional weighted translation
    Matrix3_6f jacobian_transform;
    jacobian_transform.setZero();
    if (_mean_disparity_pixels > 0) {
      // ds consider mean disparity for translation gradient - super empirical madonna
      // ds note that this coefficient is constant over all iterations and should be moved outside
      const float weight_translation =
        std::min(0.1f + disparity_pixels / _mean_disparity_pixels, 1.0f);
      jacobian_transform.block<3, 3>(0, 0) = weight_translation * Matrix3f::Identity();
    } else {
      // ds regular projective translation gradient contribution
      jacobian_transform.block<3, 3>(0, 0).setIdentity();
    }
    jacobian_transform.block<3, 3>(0, 3) = -2 * geometry3d::skew(moving);

    // ds precompute TODO move constant part of transform (rotation on first block) to pose binding
    const Matrix3_6f jacobian_camera_matrix_transform(_camera_matrix * jacobian_transform);
    assert(!jacobian_camera_matrix_transform.hasNaN());
    const float inverse_c_left          = 1 / c_left;
    const float inverse_c_right         = 1 / c_right;
    const float inverse_c_squared_left  = inverse_c_left * inverse_c_left;
    const float inverse_c_squared_right = inverse_c_right * inverse_c_right;

    // ds jacobian parts of the homogeneous division: left
    Matrix2_3f jacobian_left;
    jacobian_left << inverse_c_left, 0, -abc_left.x() * inverse_c_squared_left, 0, inverse_c_left,
      -abc_left.y() * inverse_c_squared_left;
    assert(!jacobian_left.hasNaN());

    // ds we compute only the contribution for the horizontal error: right
    Matrix1_3f jacobian_right;
    jacobian_right << inverse_c_right, 0, -abc_right.x() * inverse_c_squared_right;
    assert(!jacobian_right.hasNaN());

    // ds assemble final jacobian
    _J.setZero();

    // ds we have to compute the full block
    _J.block<2, 6>(0, 0) = jacobian_left * jacobian_camera_matrix_transform;

    // ds we only have to compute the horizontal block
    _J.block<1, 6>(2, 0) = jacobian_right * jacobian_camera_matrix_transform;
    assert(!_J.hasNaN());
  }

  INSTANTIATE(SE3RectifiedStereoProjectiveErrorFactor)
  INSTANTIATE(SE3RectifiedStereoProjectiveErrorFactorCorrespondenceDriven)

  //! error function and factor for plain 3D stereo point to plane measurements
  inline void SE3RectifiedStereoProjectiveWithSensorErrorFactor::errorAndJacobian(bool error_only) {
    assert(_image_dim.x() > 0);
    assert(_image_dim.y() > 0);
    assert(_camera_matrix.norm() > 0);
    assert(_baseline_left_in_right_pixels.norm() > 0);
    _is_valid              = false;
    const Vector3f& moving = *(this->_moving_point);
    const Vector4f& fixed  = *(this->_fixed_point);
    const Isometry3f& X    = _variables.at<0>()->estimate();
    assert(!moving.hasNaN());
    assert(!fixed.hasNaN());
    assert(!X.matrix().hasNaN());

    // ds cache disparity
    const float disparity_pixels = fixed(0) - fixed(2);
    assert(disparity_pixels >= 0);

    // ds get point into current camera frame based on variable pose
    const Vector3f point_in_robot       = X * moving;
    const Vector3f point_in_camera_left = _robot_in_camera_left * point_in_robot;

    // ds drop points with invalid depth
    if (point_in_camera_left.z() <= 0) {
      return;
    }

    // ds compute points in image plane (reprojections) - assuming rigid stereo!
    const Vector3f abc_left = _camera_matrix * point_in_camera_left;
    assert(!abc_left.hasNaN());
    const Vector3f abc_right            = abc_left + _baseline_left_in_right_pixels;
    const float& c_left                 = abc_left.z();
    const float& c_right                = abc_right.z();
    const Vector3f point_in_image_left  = abc_left / c_left;
    const Vector3f point_in_image_right = abc_right / c_right;

    // ds if the projected points are outside the image plane, skip the measurements
    if (point_in_image_left.x() < 0 || point_in_image_left.x() > _image_dim.x() ||
        point_in_image_left.y() < 0 || point_in_image_left.y() > _image_dim.y() ||
        point_in_image_right.x() < 0 || point_in_image_right.x() > _image_dim.x() ||
        point_in_image_right.y() < 0 || point_in_image_right.y() > _image_dim.y()) {
      return;
    }
    // assert(fixed(1) == fixed(3) && "images are not rectified (or correspondences invalid)");

    // ds compute error (we compute the vertical error only once, since we assume rectified cameras)
    // ds MUST BE COMPENSATED WITH DOUBLE Y WEIGHT IN OMEGA for consistency with regular stereo
    _e = Vector3f(point_in_image_left.x() - fixed(0),
                  point_in_image_left.y() - fixed(1),
                  point_in_image_right.x() - fixed(2));

    // ds factor contributes at this point
    _is_valid = true;

    // ds skip if desired
    if (error_only) {
      return;
    }

    // ds compute the jacobian of the transformation - with optional weighted translation
    Matrix3_6f jacobian_transform;
    jacobian_transform.setZero();
    if (_mean_disparity_pixels > 0) {
      // ds consider mean disparity for translation gradient - super empirical madonna
      // ds note that this coefficient is constant over all iterations and should be moved
      // outside
      const float weight_translation =
        std::min(0.1f + disparity_pixels / _mean_disparity_pixels, 1.0f);
      jacobian_transform.block<3, 3>(0, 0) = weight_translation * Matrix3f::Identity();
    } else {
      // ds regular projective translation gradient contribution
      jacobian_transform.block<3, 3>(0, 0).setIdentity();
    }
    jacobian_transform.block<3, 3>(0, 3) = -2 * geometry3d::skew(moving);

    // ds precompute TODO move constant part of transform (rotation on first block) to pose binding
    Matrix3f map_in_sensor_R = (_robot_in_camera_left * X).linear();
    const Matrix3_6f jacobian_camera_matrix_transform(_camera_matrix * map_in_sensor_R *
                                                      jacobian_transform);
    assert(!jacobian_camera_matrix_transform.hasNaN());
    const float inverse_c_left          = 1 / c_left;
    const float inverse_c_right         = 1 / c_right;
    const float inverse_c_squared_left  = inverse_c_left * inverse_c_left;
    const float inverse_c_squared_right = inverse_c_right * inverse_c_right;

    // ds jacobian parts of the homogeneous division: left
    Matrix2_3f jacobian_left;
    jacobian_left << inverse_c_left, 0, -abc_left.x() * inverse_c_squared_left, 0, inverse_c_left,
      -abc_left.y() * inverse_c_squared_left;
    assert(!jacobian_left.hasNaN());

    // ds we compute only the contribution for the horizontal error: right
    Matrix1_3f jacobian_right;
    jacobian_right << inverse_c_right, 0, -abc_right.x() * inverse_c_squared_right;
    assert(!jacobian_right.hasNaN());

    // ds assemble final jacobian
    _J.setZero();

    // ds we have to compute the full block
    _J.block<2, 6>(0, 0) = jacobian_left * jacobian_camera_matrix_transform;

    // ds we only have to compute the horizontal block
    _J.block<1, 6>(2, 0) = jacobian_right * jacobian_camera_matrix_transform;
    assert(!_J.hasNaN());
  }

  INSTANTIATE(SE3RectifiedStereoProjectiveWithSensorErrorFactor)
  INSTANTIATE(SE3RectifiedStereoProjectiveWithSensorErrorFactorCorrespondenceDriven)

} // namespace srrg2_solver
