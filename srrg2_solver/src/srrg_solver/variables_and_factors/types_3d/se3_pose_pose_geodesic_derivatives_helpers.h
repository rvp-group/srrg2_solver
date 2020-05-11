#pragma once
#include <srrg_geometry/geometry_defs.h>

namespace srrg2_solver {

  /**
   * @brief Helper class for jacobian computation of
   * SE3 Geodesic Error Function
   */
  template <typename Scalar_>
  class SE3PosePoseGeodesicErrorFactorJacobianHelper {
  public:
    using Scalar                      = Scalar_;
    using IsometryType                = Eigen::Transform<Scalar_, 3, Eigen::Isometry>;
    using PartialDerivativeMatrixType = Eigen::Matrix<Scalar, 3, 9, Eigen::ColMajor>;
    using JacobianMatrixType          = Eigen::Matrix<Scalar, 6, 6, Eigen::ColMajor>;

    /**
     * @brief ctor: object life - everything is super static
     */
    SE3PosePoseGeodesicErrorFactorJacobianHelper() {
    }

    /**
     * @brief dtor: object life - everything is super static
     */
    ~SE3PosePoseGeodesicErrorFactorJacobianHelper() {
    }
    /**
     * @brief computes the Jacobians for the standard Geodesic error function
     *        error vector should be [x y z qx qy qz].
     * @param[in] inverse_Z_T_  : inverse measurement
     * @param[in] prediction_T_ : h(X) computed as from_T.inverse() * to_T
     * @param[in] error_T_      : h(X) - Z computed as Z.inverse() * prediction_T
     * @param[out] Ji_ : jacobian wrt vertex[0]
     * @param[out] Jj_ : jacobian wrt vertex[1]
     */
    static void computeGeodesicJacobians(const IsometryType& inverse_Z_T_,
                                         const IsometryType& prediction_T_,
                                         const IsometryType& error_T_,
                                         JacobianMatrixType& Ji_,
                                         JacobianMatrixType& Jj_);

  private:
    /**
     * @brief computes the derivative case for dq/dR
     * @param[out] S : auxiliary variable to pass to other functions
     * @param[out] qw: w-th quaterion element
     * @param[in] r{00-22}: elements of the rotation matrix
     */
    static inline size_t q2m(Scalar& S,
                             Scalar& qw,
                             const Scalar& r00,
                             const Scalar& r10,
                             const Scalar& r20,
                             const Scalar& r01,
                             const Scalar& r11,
                             const Scalar& r21,
                             const Scalar& r02,
                             const Scalar& r12,
                             const Scalar& r22) {
      const Scalar tr = r00 + r11 + r22;
      if (tr > 0) {
        S  = std::sqrt(tr + 1.0) * 2; // S=4*qw
        qw = 0.25 * S;
        return 0;
      } else if ((r00 > r11) & (r00 > r22)) {
        S  = std::sqrt(1.0 + r00 - r11 - r22) * 2; // S=4*qx
        qw = (r21 - r12) / S;
        return 1;
      } else if (r11 > r22) {
        S  = std::sqrt(1.0 + r11 - r00 - r22) * 2; // S=4*qy
        qw = (r02 - r20) / S;
        return 2;
      } else {
        S  = std::sqrt(1.0 + r22 - r00 - r11) * 2; // S=4*qz
        qw = (r10 - r01) / S;
        return 3;
      }
    }

    /**
     * @brief computes the partial derivative of dq/dR
     * @param[out] dq_dR : partial derivative of dq/dR
     * @param[in] r{11-33}: elements of the rotation matrix
     */
    static void compute_dq_dR(PartialDerivativeMatrixType& dq_dR,
                              const Scalar& r11,
                              const Scalar& r21,
                              const Scalar& r31,
                              const Scalar& r12,
                              const Scalar& r22,
                              const Scalar& r32,
                              const Scalar& r13,
                              const Scalar& r23,
                              const Scalar& r33) {
      Scalar qw                  = 0.0;
      Scalar S                   = 0.0;
      const size_t derivate_case = q2m(S, qw, r11, r21, r31, r12, r22, r32, r13, r23, r33);
      S *= .25;
      switch (derivate_case) {
        case 0:
          compute_dq_dR_w(dq_dR, S, r11, r21, r31, r12, r22, r32, r13, r23, r33);
          break;
        case 1:
          compute_dq_dR_x(dq_dR, S, r11, r21, r31, r12, r22, r32, r13, r23, r33);
          break;
        case 2:
          compute_dq_dR_y(dq_dR, S, r11, r21, r31, r12, r22, r32, r13, r23, r33);
          break;
        case 3:
          compute_dq_dR_z(dq_dR, S, r11, r21, r31, r12, r22, r32, r13, r23, r33);
          break;
      }
      if (qw <= 0) {
        dq_dR *= -1;
      }
    }

    /**
     * @brief computes the *qw* component of dq/dR
     * @param[out] dq_dR_w : *qw* component of dq/dR
     * @param[in] qw: w-th quaterion element
     * @param[in] r{00-22}: elements of the rotation matrix
     */
    static inline void compute_dq_dR_w(PartialDerivativeMatrixType& dq_dR_w,
                                       const Scalar& qw,
                                       const Scalar& r00,
                                       const Scalar& r10,
                                       const Scalar& r20,
                                       const Scalar& r01,
                                       const Scalar& r11,
                                       const Scalar& r21,
                                       const Scalar& r02,
                                       const Scalar& r12,
                                       const Scalar& r22) {
      (void) r00;
      (void) r11;
      (void) r22;
      Scalar _aux1  = 1 / std::pow(qw, 3);
      Scalar _aux2  = -0.03125 * (r21 - r12) * _aux1;
      Scalar _aux3  = 1 / qw;
      Scalar _aux4  = 0.25 * _aux3;
      Scalar _aux5  = -0.25 * _aux3;
      Scalar _aux6  = 0.03125 * (r20 - r02) * _aux1;
      Scalar _aux7  = -0.03125 * (r10 - r01) * _aux1;
      dq_dR_w(0, 0) = _aux2;
      dq_dR_w(0, 1) = 0;
      dq_dR_w(0, 2) = 0;
      dq_dR_w(0, 3) = 0;
      dq_dR_w(0, 4) = _aux2;
      dq_dR_w(0, 5) = _aux4;
      dq_dR_w(0, 6) = 0;
      dq_dR_w(0, 7) = _aux5;
      dq_dR_w(0, 8) = _aux2;
      dq_dR_w(1, 0) = _aux6;
      dq_dR_w(1, 1) = 0;
      dq_dR_w(1, 2) = _aux5;
      dq_dR_w(1, 3) = 0;
      dq_dR_w(1, 4) = _aux6;
      dq_dR_w(1, 5) = 0;
      dq_dR_w(1, 6) = _aux4;
      dq_dR_w(1, 7) = 0;
      dq_dR_w(1, 8) = _aux6;
      dq_dR_w(2, 0) = _aux7;
      dq_dR_w(2, 1) = _aux4;
      dq_dR_w(2, 2) = 0;
      dq_dR_w(2, 3) = _aux5;
      dq_dR_w(2, 4) = _aux7;
      dq_dR_w(2, 5) = 0;
      dq_dR_w(2, 6) = 0;
      dq_dR_w(2, 7) = 0;
      dq_dR_w(2, 8) = _aux7;
    }

    /**
     * @brief computes the *qx* component of dq/dR
     * @param[out] dq_dR_x : *qx* component of dq/dR
     * @param[in] qx: x-th quaterion element
     * @param[in] r{00-22}: elements of the rotation matrix
     */
    static inline void compute_dq_dR_x(PartialDerivativeMatrixType& dq_dR_x,
                                       const Scalar& qx,
                                       const Scalar& r00,
                                       const Scalar& r10,
                                       const Scalar& r20,
                                       const Scalar& r01,
                                       const Scalar& r11,
                                       const Scalar& r21,
                                       const Scalar& r02,
                                       const Scalar& r12,
                                       const Scalar& r22) {
      (void) r00;
      (void) r11;
      (void) r21;
      (void) r12;
      (void) r22;
      Scalar _aux1  = 1 / qx;
      Scalar _aux2  = -0.125 * _aux1;
      Scalar _aux3  = 1 / std::pow(qx, 3);
      Scalar _aux4  = r10 + r01;
      Scalar _aux5  = 0.25 * _aux1;
      Scalar _aux6  = 0.03125 * _aux3 * _aux4;
      Scalar _aux7  = r20 + r02;
      Scalar _aux8  = 0.03125 * _aux3 * _aux7;
      dq_dR_x(0, 0) = 0.125 * _aux1;
      dq_dR_x(0, 1) = 0;
      dq_dR_x(0, 2) = 0;
      dq_dR_x(0, 3) = 0;
      dq_dR_x(0, 4) = _aux2;
      dq_dR_x(0, 5) = 0;
      dq_dR_x(0, 6) = 0;
      dq_dR_x(0, 7) = 0;
      dq_dR_x(0, 8) = _aux2;
      dq_dR_x(1, 0) = -0.03125 * _aux3 * _aux4;
      dq_dR_x(1, 1) = _aux5;
      dq_dR_x(1, 2) = 0;
      dq_dR_x(1, 3) = _aux5;
      dq_dR_x(1, 4) = _aux6;
      dq_dR_x(1, 5) = 0;
      dq_dR_x(1, 6) = 0;
      dq_dR_x(1, 7) = 0;
      dq_dR_x(1, 8) = _aux6;
      dq_dR_x(2, 0) = -0.03125 * _aux3 * _aux7;
      dq_dR_x(2, 1) = 0;
      dq_dR_x(2, 2) = _aux5;
      dq_dR_x(2, 3) = 0;
      dq_dR_x(2, 4) = _aux8;
      dq_dR_x(2, 5) = 0;
      dq_dR_x(2, 6) = _aux5;
      dq_dR_x(2, 7) = 0;
      dq_dR_x(2, 8) = _aux8;
    }

    /**
     * @brief computes the *qy* component of dq/dR
     * @param[out] dq_dR_y : *qy* component of dq/dR
     * @param[in] qy: y-th quaterion element
     * @param[in] r{00-22}: elements of the rotation matrix
     */
    static inline void compute_dq_dR_y(PartialDerivativeMatrixType& dq_dR_y,
                                       const Scalar& qy,
                                       const Scalar& r00,
                                       const Scalar& r10,
                                       const Scalar& r20,
                                       const Scalar& r01,
                                       const Scalar& r11,
                                       const Scalar& r21,
                                       const Scalar& r02,
                                       const Scalar& r12,
                                       const Scalar& r22) {
      (void) r00;
      (void) r20;
      (void) r11;
      (void) r02;
      (void) r22;
      Scalar _aux1  = 1 / std::pow(qy, 3);
      Scalar _aux2  = r10 + r01;
      Scalar _aux3  = 0.03125 * _aux1 * _aux2;
      Scalar _aux4  = 1 / qy;
      Scalar _aux5  = 0.25 * _aux4;
      Scalar _aux6  = -0.125 * _aux4;
      Scalar _aux7  = r21 + r12;
      Scalar _aux8  = 0.03125 * _aux1 * _aux7;
      dq_dR_y(0, 0) = _aux3;
      dq_dR_y(0, 1) = _aux5;
      dq_dR_y(0, 2) = 0;
      dq_dR_y(0, 3) = _aux5;
      dq_dR_y(0, 4) = -0.03125 * _aux1 * _aux2;
      dq_dR_y(0, 5) = 0;
      dq_dR_y(0, 6) = 0;
      dq_dR_y(0, 7) = 0;
      dq_dR_y(0, 8) = _aux3;
      dq_dR_y(1, 0) = _aux6;
      dq_dR_y(1, 1) = 0;
      dq_dR_y(1, 2) = 0;
      dq_dR_y(1, 3) = 0;
      dq_dR_y(1, 4) = 0.125 * _aux4;
      dq_dR_y(1, 5) = 0;
      dq_dR_y(1, 6) = 0;
      dq_dR_y(1, 7) = 0;
      dq_dR_y(1, 8) = _aux6;
      dq_dR_y(2, 0) = _aux8;
      dq_dR_y(2, 1) = 0;
      dq_dR_y(2, 2) = 0;
      dq_dR_y(2, 3) = 0;
      dq_dR_y(2, 4) = -0.03125 * _aux1 * _aux7;
      dq_dR_y(2, 5) = _aux5;
      dq_dR_y(2, 6) = 0;
      dq_dR_y(2, 7) = _aux5;
      dq_dR_y(2, 8) = _aux8;
    }

    /**
     * @brief computes the *qz* component of dq/dR
     * @param[out] dq_dR_z : *qz* component of dq/dR
     * @param[in] qz: z-th quaterion element
     * @param[in] r{00-22}: elements of the rotation matrix
     */
    static inline void compute_dq_dR_z(PartialDerivativeMatrixType& dq_dR_z,
                                       const Scalar& qz,
                                       const Scalar& r00,
                                       const Scalar& r10,
                                       const Scalar& r20,
                                       const Scalar& r01,
                                       const Scalar& r11,
                                       const Scalar& r21,
                                       const Scalar& r02,
                                       const Scalar& r12,
                                       const Scalar& r22) {
      (void) r00;
      (void) r10;
      (void) r01;
      (void) r11;
      (void) r22;
      Scalar _aux1  = 1 / std::pow(qz, 3);
      Scalar _aux2  = r20 + r02;
      Scalar _aux3  = 0.03125 * _aux1 * _aux2;
      Scalar _aux4  = 1 / qz;
      Scalar _aux5  = 0.25 * _aux4;
      Scalar _aux6  = r21 + r12;
      Scalar _aux7  = 0.03125 * _aux1 * _aux6;
      Scalar _aux8  = -0.125 * _aux4;
      dq_dR_z(0, 0) = _aux3;
      dq_dR_z(0, 1) = 0;
      dq_dR_z(0, 2) = _aux5;
      dq_dR_z(0, 3) = 0;
      dq_dR_z(0, 4) = _aux3;
      dq_dR_z(0, 5) = 0;
      dq_dR_z(0, 6) = _aux5;
      dq_dR_z(0, 7) = 0;
      dq_dR_z(0, 8) = -0.03125 * _aux1 * _aux2;
      dq_dR_z(1, 0) = _aux7;
      dq_dR_z(1, 1) = 0;
      dq_dR_z(1, 2) = 0;
      dq_dR_z(1, 3) = 0;
      dq_dR_z(1, 4) = _aux7;
      dq_dR_z(1, 5) = _aux5;
      dq_dR_z(1, 6) = 0;
      dq_dR_z(1, 7) = _aux5;
      dq_dR_z(1, 8) = -0.03125 * _aux1 * _aux6;
      dq_dR_z(2, 0) = _aux8;
      dq_dR_z(2, 1) = 0;
      dq_dR_z(2, 2) = 0;
      dq_dR_z(2, 3) = 0;
      dq_dR_z(2, 4) = _aux8;
      dq_dR_z(2, 5) = 0;
      dq_dR_z(2, 6) = 0;
      dq_dR_z(2, 7) = 0;
      dq_dR_z(2, 8) = 0.125 * _aux4;
    }

    /**
     * @brief computes dR/dq
     * @param[out] dR_dq : dq/dR
     * @param[in] qx: x-th quaterion element
     * @param[in] qy: y-th quaterion element
     * @param[in] qz: z-th quaterion element
     * @param[in] qw: w-th quaterion element
     */
    static inline void compute_dR_dq(PartialDerivativeMatrixType& dR_dq,
                                     const Scalar& qx,
                                     const Scalar& qy,
                                     const Scalar& qz,
                                     const Scalar& qw) {
      Scalar _aux1  = -4 * qy;
      Scalar _aux2  = -4 * qz;
      Scalar _aux3  = 1 / qw;
      Scalar _aux4  = 2 * qx * qz;
      Scalar _aux5  = -_aux3 * (_aux4 - 2 * qw * qy);
      Scalar _aux6  = 2 * qy * qz;
      Scalar _aux7  = -_aux3 * (_aux6 - 2 * qw * qx);
      Scalar _aux8  = -2 * std::pow(qw, 2);
      Scalar _aux9  = _aux8 + 2 * std::pow(qz, 2);
      Scalar _aux10 = 2 * qw * qz;
      Scalar _aux11 = (_aux10 + 2 * qx * qy) * _aux3;
      Scalar _aux12 = _aux8 + 2 * std::pow(qy, 2);
      Scalar _aux13 = _aux3 * (_aux6 + 2 * qw * qx);
      Scalar _aux14 = _aux3 * (_aux4 + 2 * qw * qy);
      Scalar _aux15 = -4 * qx;
      Scalar _aux16 = _aux8 + 2 * std::pow(qx, 2);
      Scalar _aux17 = (_aux10 - 2 * qx * qy) * _aux3;
      dR_dq(0, 0)   = 0;
      dR_dq(0, 1)   = _aux1;
      dR_dq(0, 2)   = _aux2;
      dR_dq(1, 0)   = _aux5;
      dR_dq(1, 1)   = _aux7;
      dR_dq(1, 2)   = -_aux3 * _aux9;
      dR_dq(2, 0)   = _aux11;
      dR_dq(2, 1)   = _aux12 * _aux3;
      dR_dq(2, 2)   = _aux13;
      dR_dq(3, 0)   = _aux14;
      dR_dq(3, 1)   = _aux13;
      dR_dq(3, 2)   = _aux3 * _aux9;
      dR_dq(4, 0)   = _aux15;
      dR_dq(4, 1)   = 0;
      dR_dq(4, 2)   = _aux2;
      dR_dq(5, 0)   = -_aux16 * _aux3;
      dR_dq(5, 1)   = _aux17;
      dR_dq(5, 2)   = _aux5;
      dR_dq(6, 0)   = _aux17;
      dR_dq(6, 1)   = -_aux12 * _aux3;
      dR_dq(6, 2)   = _aux7;
      dR_dq(7, 0)   = _aux16 * _aux3;
      dR_dq(7, 1)   = _aux11;
      dR_dq(7, 2)   = _aux14;
      dR_dq(8, 0)   = _aux15;
      dR_dq(8, 1)   = _aux1;
      dR_dq(8, 2)   = 0;
    }

    /**
     * @brief skew helper function
     * @param[out] s : skew matrix
     * @param[in] v: vector to be skew'd
     */
    template <typename Derived, typename DerivedOther>
    static inline void customSkewT(Eigen::MatrixBase<Derived>& s,
                                   const Eigen::MatrixBase<DerivedOther>& v) {
      const Scalar x = 2 * v(0);
      const Scalar y = 2 * v(1);
      const Scalar z = 2 * v(2);
      s << 0., -z, y, z, 0, -x, -y, x, 0;
    }
    template <typename Derived, typename DerivedOther>
    static inline void customSkewT(Eigen::MatrixBase<Derived>& Sx,
                                   Eigen::MatrixBase<Derived>& Sy,
                                   Eigen::MatrixBase<Derived>& Sz,
                                   const Eigen::MatrixBase<DerivedOther>& R) {
      const Scalar r11 = 2 * R(0, 0), r12 = 2 * R(0, 1), r13 = 2 * R(0, 2), r21 = 2 * R(1, 0),
                   r22 = 2 * R(1, 1), r23 = 2 * R(1, 2), r31 = 2 * R(2, 0), r32 = 2 * R(2, 1),
                   r33 = 2 * R(2, 2);
      Sx << 0, 0, 0, r31, r32, r33, -r21, -r22, -r23;
      Sy << -r31, -r32, -r33, 0, 0, 0, r11, r12, r13;
      Sz << r21, r22, r23, -r11, -r12, -r13, 0, 0, 0;
    }

    /**
     * @brief piece-wise skew helper function
     * @param[out] Sx : x-related skew matrix
     * @param[out] Sy : y-related skew matrix
     * @param[out] Sz : z-related skew matrix
     * @param[in] R: Rotation matrix to be skew'd
     */
    template <typename Derived, typename DerivedOther>
    static void customSkew(Eigen::MatrixBase<Derived>& Sx,
                           Eigen::MatrixBase<Derived>& Sy,
                           Eigen::MatrixBase<Derived>& Sz,
                           const Eigen::MatrixBase<DerivedOther>& R) {
      const Scalar r11 = 2 * R(0, 0), r12 = 2 * R(0, 1), r13 = 2 * R(0, 2), r21 = 2 * R(1, 0),
                   r22 = 2 * R(1, 1), r23 = 2 * R(1, 2), r31 = 2 * R(2, 0), r32 = 2 * R(2, 1),
                   r33 = 2 * R(2, 2);
      Sx << 0, 0, 0, -r31, -r32, -r33, r21, r22, r23;
      Sy << r31, r32, r33, 0, 0, 0, -r11, -r12, -r13;
      Sz << -r21, -r22, -r23, r11, r12, r13, 0, 0, 0;
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace srrg2_solver

#include "se3_pose_pose_geodesic_derivatives_helpers.hpp"
