#include "se3_projective_error_factor.h"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {

  inline void SE3ProjectiveErrorFactor::errorAndJacobian(bool error_only) {
    assert(_image_dim.x() > 0);
    assert(_image_dim.y() > 0);
    assert(_camera_matrix.norm() > 0);
    _is_valid              = false;
    const Vector3f& moving = *_moving_point;
    const Vector2f& fixed  = *_fixed_point;
    const Isometry3f& X    = _variables.at<0>()->estimate();

    const Vector3f robot_pt = X * moving;
    const float& z          = robot_pt.z();
    if (z < 0) {
      std::cerr << "z: " << z << std::endl;
      return;
    }

    const float iz        = 1. / z;
    const Vector3f cam_pt = _camera_matrix * robot_pt;
    const Vector2f img_pt(cam_pt.x() * iz, cam_pt.y() * iz);

    if (img_pt.x() < 0 || img_pt.x() > _image_dim.x()) {
      return;
    }
    if (img_pt.y() < 0 || img_pt.y() > _image_dim.y()) {
      return;
    }
    _e        = img_pt - fixed;
    _is_valid = true;
    if (error_only) {
      return;
    }

    /*
      Eigen::Matrix<float, 3, 6> J_icp;
      J_icp.setZero();
      J_icp.block<3,3>(0,0)=R;
      J_icp.block<3,3>(0,3)=-R*geometry3d::skew(moving);
    */

    float iz2 = iz * iz;
    Matrix2_3f J_proj;
    J_proj << iz, 0, -cam_pt.x() * iz2, 0, iz, -cam_pt.y() * iz2;

    Matrix2_3f Jpkr      = J_proj * _camera_matrix;
    _J.block<2, 3>(0, 0) = Jpkr;
    _J.block<2, 3>(0, 3) = -2 * Jpkr * geometry3d::skew(moving);
  }

  INSTANTIATE(SE3ProjectiveErrorFactor)
  INSTANTIATE(SE3ProjectiveErrorFactorCorrespondenceDriven)

  inline void SE3ProjectiveWithSensorErrorFactor::errorAndJacobian(bool error_only) {
    assert(_image_dim.x() > 0);
    assert(_image_dim.y() > 0);
    assert(_camera_matrix.norm() > 0);
    _is_valid              = false;
    const Vector3f& moving = *_moving_point;
    const Vector2f& fixed  = *_fixed_point;

    const Isometry3f& X            = _variables.at<0>()->estimate();
    const Vector3f point_in_robot  = X * moving;
    const Vector3f point_in_camera = _robot_in_sensor * point_in_robot;
    const float& z                 = point_in_camera.z();
    if (z < 0) {
      return;
    }

    const float iz        = 1. / z;
    const Vector3f cam_pt = _camera_matrix * point_in_camera;
    const Vector2f img_pt(cam_pt.x() * iz, cam_pt.y() * iz);

    if (img_pt.x() < 0 || img_pt.x() > _image_dim.x()) {
      return;
    }
    if (img_pt.y() < 0 || img_pt.y() > _image_dim.y()) {
      return;
    }
    _e        = img_pt - fixed;
    _is_valid = true;
    if (error_only) {
      return;
    }

    /*
      Eigen::Matrix<float, 3, 6> J_icp;
      J_icp.setZero();
      J_icp.block<3,3>(0,0)=R;
      J_icp.block<3,3>(0,3)=-R*geometry3d::skew(moving);
    */

    float iz2 = iz * iz;
    Matrix2_3f J_proj;
    // clang-format off
    J_proj << iz,  0, -cam_pt.x() * iz2,
               0, iz, -cam_pt.y() * iz2;
    // clang-format on

    Matrix3_6f J_transform;
    // ds translation
    J_transform.block<3, 3>(0, 0).setIdentity();
    // ds rotation (euler)
    J_transform.block<3, 3>(0, 3) = -2 * geometry3d::skew(moving);

    // TODO check
    Matrix3f map_in_sensor_R = (_robot_in_sensor * X).linear();

    _J = J_proj * _camera_matrix * map_in_sensor_R * J_transform;

    /* srrg it was
     Matrix2_3f Jpkr      = J_proj * _camera_matrix;
    _J.block<2, 3>(0, 0) = Jpkr;
    _J.block<2, 3>(0, 3) = -2 * Jpkr * geometry3d::skew(robot_pt);
     */
  }

  INSTANTIATE(SE3ProjectiveWithSensorErrorFactor)
  INSTANTIATE(SE3ProjectiveWithSensorErrorFactorCorrespondenceDriven)

} // namespace srrg2_solver
