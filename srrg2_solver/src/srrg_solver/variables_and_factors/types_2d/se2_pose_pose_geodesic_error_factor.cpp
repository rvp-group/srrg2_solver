#include "se2_pose_pose_geodesic_error_factor.h"


//! include this: this contains all the implementations of the factors
//! that are hidden to the modules that do not need them to avoid excessive compilation times (EVIL)
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {

  using namespace srrg2_core;

  void SE2PosePoseGeodesicErrorFactor::errorAndJacobian(bool error_only_) {
    VariableSE2Right* v_from = _variables.at<0>();
    VariableSE2Right* v_to   = _variables.at<1>();

    const Isometry2f& from           = v_from->estimate();
    const Isometry2f& to             = v_to->estimate();
    const Matrix2f R_from_transpose  = from.linear().transpose();
    const Matrix2f& R_to             = to.linear();
    const Matrix2f& R_z_transpose    = _inverse_measured_relative_pose.linear();
    const Vector2f delta_translation = to.translation() - from.translation();
    const Isometry2f prediction      = from.inverse() * to;
    const Isometry2f error_SE2       = _inverse_measured_relative_pose * prediction;
    _e                               = geometry2d::t2v(error_SE2);
    if (error_only_) {
      return;
    }

    // tg derivative of the atan2 with respect to the involved terms in the rotation matrix
    // t2v[2] = atan2(R(1,0)/R(1,1))
    const Matrix2f& R_error  = error_SE2.linear();
    const Vector2f row_error = R_error.row(1).transpose();
    // tg datan2/d[R10, R11] = 1/(R10^2 + R11^2) * [R11, -R10]
    Vector2f partial_atan2_R = geometry2d::skew(row_error);
    const float scale        = row_error.squaredNorm();
    partial_atan2_R /= scale;

    // tg derivatives of R(delta_theta) and R(-delta_theta) with respect to delta_theta(=0)
    Matrix2f skew_R_transpose;
    skew_R_transpose << 0, 1, -1, 0;
    Matrix2f skew_R;
    skew_R << 0, -1, 1, 0;

    auto J_from = jacobian<0>();
    J_from.setZero();
    // tg jacobian of the translation with respect to the full perturbation
    J_from.block<2, 2>(0, 0) = -R_z_transpose;
    J_from.block<2, 1>(0, 2) =
      R_z_transpose * skew_R_transpose * R_from_transpose * delta_translation;
    // tg derivative R_error with respect to delta_theta_from (=0)
    Matrix2f R_aux_from = R_z_transpose * skew_R_transpose * R_from_transpose * R_to;
    Vector2f R_row      = R_aux_from.row(1).transpose();
    J_from(2, 2)        = partial_atan2_R.dot(R_row);

    auto J_to = jacobian<1>();
    J_to.setZero();
    // tg jacobian of the translation with respect to the full perturbation
    J_to.block<2, 2>(0, 0) = R_error;
    // tg derivative R_error with respect to delta_theta_to (=0)
    Matrix2f R_aux_to = R_error * skew_R;
    R_row             = R_aux_to.row(1).transpose();
    J_to(2, 2)        = partial_atan2_R.dot(R_row);
  }

  void SE2PosePoseGeodesicErrorFactor::_drawImpl(srrg2_core::ViewerCanvasPtr canvas) const {
    if (!canvas) {
      return;
    }

    Vector3f segments[2];
    const VariableSE2Right* from_v = dynamic_cast<const VariableSE2Right*>(variable(0));
    const VariableSE2Right* to_v   = dynamic_cast<const VariableSE2Right*>(variable(1));
    if (!from_v || !to_v) {
      return;
    }

    const Isometry2f& from = from_v->estimate();
    const Isometry2f& to   = to_v->estimate();
    segments[0] << from.translation().x(), from.translation().y(), 0.f;
    segments[1] << to.translation().x(), to.translation().y(), 0.f;
    canvas->putSegment(2, segments, 0);
  }

  INSTANTIATE(SE2PosePoseGeodesicErrorFactor)

} // namespace srrg2_solver
