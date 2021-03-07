#include "se3_pose_pose_chordal_error_factor.h"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  void SE3PosePoseChordalEulerLeftErrorFactor::errorAndJacobian(bool error_only_) {
    using JacobianMatrixType = Eigen::Matrix<Scalar, 12, 6>;

    // ia take objects - static cast for speeeeeed
    VariableSE3EulerLeft* v_from = _variables.at<0>();
    VariableSE3EulerLeft* v_to   = _variables.at<1>();
    assert(v_from && v_to &&
           "SE3PosePoseChordalEulerErrorFactor::errorAndJacobian|invalid variables type");

    const Isometry3_<Scalar>& pose_i      = v_from->estimate();
    const Isometry3_<Scalar>& pose_j      = v_to->estimate();
    const Isometry3_<Scalar> prediction_T = pose_i.inverse() * pose_j;

    // ia compute the error
    Isometry3_<Scalar> error_T = Isometry3_<Scalar>::Identity();
    error_T.matrix()           = prediction_T.matrix() - _measurement.matrix();
    _e                         = geometry3d::flattenByCols(error_T);

    if (error_only_) {
      return;
    }

    // ia compute the jacobians
    srrg2_core::Matrix3_<Scalar> Rx0, Ry0, Rz0;
    Rx0 << 0, 0, 0, 0, 0, -1, 0, 1, 0;
    Ry0 << 0, 0, 1, 0, 0, 0, -1, 0, 0;
    Rz0 << 0, -1, 0, 1, 0, 0, 0, 0, 0;

    const Matrix3_<Scalar>& Ri = pose_i.linear();
    const Matrix3_<Scalar>& Rj = pose_j.linear();
    const Vector3_<Scalar>& tj = pose_j.translation();

    const Matrix3_<Scalar> dR_x = Ri.transpose() * Rx0 * Rj;
    const Matrix3_<Scalar> dR_y = Ri.transpose() * Ry0 * Rj;
    const Matrix3_<Scalar> dR_z = Ri.transpose() * Rz0 * Rj;

    srrg2_core::Vector9_<Scalar> dr_x_flattened, dr_y_flattened, dr_z_flattened;
    dr_x_flattened << dR_x.col(0), dR_x.col(1), dR_x.col(2);
    dr_y_flattened << dR_y.col(0), dR_y.col(1), dR_y.col(2);
    dr_z_flattened << dR_z.col(0), dR_z.col(1), dR_z.col(2);

    //! Fill Jj
    JacobianMatrixType Jj = JacobianMatrixType::Zero();

    Jj.block<9, 1>(0, 3) = dr_x_flattened;
    Jj.block<9, 1>(0, 4) = dr_y_flattened;
    Jj.block<9, 1>(0, 5) = dr_z_flattened;
    Jj.block<3, 3>(9, 0) = Ri.transpose();
    Jj.block<3, 3>(9, 3) = -Ri.transpose() * geometry3d::skew(tj);

    // ia assign the jacobians
    jacobian<0>() = -Jj;
    jacobian<1>() = Jj;

    return;
  }

  void SE3PosePoseChordalEulerLeftErrorFactor::_drawImpl(ViewerCanvasPtr canvas_) const {
    if (!canvas_) {
      throw std::runtime_error("SE3PosePoseChordalEulerErrorFactor::draw|invalid canvas");
    }
    Vector3f coords[2];
    coords[0] = static_cast<const VariableSE3EulerLeft*>(variable(0))->estimate().translation();
    coords[1] = static_cast<const VariableSE3EulerLeft*>(variable(1))->estimate().translation();
    canvas_->pushColor();
    canvas_->setColor(srrg2_core::ColorPalette::color3fBlue());
    canvas_->putLine(2, coords);
    canvas_->popAttribute();
  }

  INSTANTIATE(SE3PosePoseChordalEulerLeftErrorFactor)

} // namespace srrg2_solver
