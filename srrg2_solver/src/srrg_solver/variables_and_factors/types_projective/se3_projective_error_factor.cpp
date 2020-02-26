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
    _J.block<2, 3>(0, 3) = -2 * Jpkr * geometry3d::skew(robot_pt);
  }

  inline void SE3ProjectiveWithSensorErrorFactor::errorAndJacobian(bool error_only) {
    assert(_image_dim.x() > 0);
    assert(_image_dim.y() > 0);
    assert(_camera_matrix.norm() > 0);
    _is_valid              = false;
    const Vector3f& moving = *_moving_point;
    const Vector2f& fixed  = *_fixed_point;
    const Isometry3f X     = _variables.at<0>()->estimate() * _sensor_in_robot;

    const Vector3f robot_pt = X * moving;
    const float& z          = robot_pt.z();
    if (z < 0) {
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
    _J.block<2, 3>(0, 3) = -2 * Jpkr * geometry3d::skew(robot_pt);
  }

  INSTANTIATE(SE3ProjectiveWithSensorErrorFactor)
  INSTANTIATE(SE3ProjectiveWithSensorErrorFactorCorrespondenceDriven)

  INSTANTIATE(SE3ProjectiveErrorFactor)
  INSTANTIATE(SE3ProjectiveErrorFactorCorrespondenceDriven)

} // namespace srrg2_solver
