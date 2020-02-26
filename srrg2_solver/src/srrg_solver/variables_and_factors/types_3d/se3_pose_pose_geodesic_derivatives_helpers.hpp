#pragma once
#include <srrg_geometry/geometry3d.h>

namespace srrg2_solver {

  template <typename Scalar_>
  void SE3PosePoseGeodesicErrorFactorJacobianHelper<Scalar_>::computeGeodesicJacobians(
    const IsometryType& inverse_Z_T_,
    const IsometryType& prediction_T_,
    const IsometryType& error_T_,
    JacobianMatrixType& Ji_,
    JacobianMatrixType& Jj_) {
    using Matrix3 = srrg2_core::Matrix3_<Scalar>;

    // ia compute the goddamn jacobian - evil starts here
    Ji_.setZero();
    Jj_.setZero();

    const auto inverse_meas_R = inverse_Z_T_.linear();       // ia Ra
    const auto predition_R    = prediction_T_.linear();      // ia Rb
    const auto error_R        = error_T_.linear();           // ia Re
    const auto prediction_t   = prediction_T_.translation(); // ia tb

    PartialDerivativeMatrixType dq_dR = PartialDerivativeMatrixType::Zero();
    compute_dq_dR(dq_dR,
                  error_R(0, 0),
                  error_R(1, 0),
                  error_R(2, 0),
                  error_R(0, 1),
                  error_R(1, 1),
                  error_R(2, 1),
                  error_R(0, 2),
                  error_R(1, 2),
                  error_R(2, 2));

    // dte/dti
    Ji_.template block<3, 3>(0, 0) = -inverse_meas_R;

    // dte/dtj
    Jj_.template block<3, 3>(0, 0) = error_R;

    // dte/dqi
    {
      Matrix3 S;
      customSkewT(S, prediction_t);
      Ji_.template block<3, 3>(0, 3) = inverse_meas_R * S;
    }

    // dte/dqj: this is zero

    // ia super evil here
    Scalar buf[27];
    Eigen::Map<Eigen::Matrix<Scalar, 9, 3, Eigen::ColMajor>> M(buf);
    Matrix3 Sxt, Syt, Szt;
    // dre/dqi
    {
      customSkewT(Sxt, Syt, Szt, predition_R);
      Eigen::Map<Matrix3> Mx(buf);
      Mx.noalias() = inverse_meas_R * Sxt;
      Eigen::Map<Matrix3> My(buf + 9);
      My.noalias() = inverse_meas_R * Syt;
      Eigen::Map<Matrix3> Mz(buf + 18);
      Mz.noalias() = inverse_meas_R * Szt;

      Ji_.template block<3, 3>(3, 3) = dq_dR * M;
    }

    // dre/dqj
    {
      Matrix3& Sx = Sxt;
      Matrix3& Sy = Syt;
      Matrix3& Sz = Szt;
      customSkew(Sx, Sy, Sz, Matrix3::Identity());
      Eigen::Map<Matrix3> Mx(buf);
      Mx.noalias() = error_R * Sx;
      Eigen::Map<Matrix3> My(buf + 9);
      My.noalias() = error_R * Sy;
      Eigen::Map<Matrix3> Mz(buf + 18);
      Mz.noalias() = error_R * Sz;

      Jj_.template block<3, 3>(3, 3) = dq_dR * M;
    }
  }

} // namespace srrg2_solver
