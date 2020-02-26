#include "se2_prior_error_factor.h"

//! include this: this contains all the implementations of the factors
//! that are hidden to the modules that do not need them to avoid excessive compilation times (EVIL)
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  void SE2PriorErrorFactor::errorAndJacobian(bool error_only_) {
    const MeasurementType& X        = _variables.at<0>()->estimate();
    const MeasurementType error_SE2 = _inverse_measurement * X;
    _e                              = geometry2d::t2v(error_SE2);
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
    Matrix2f skew_R;
    skew_R << 0, -1, 1, 0;
    _J.block<2, 2>(0, 0) = R_error;
    // tg derivative R_error with respect to delta_theta (=0)
    Matrix2f R_aux_to = R_error * skew_R;
    Vector2f R_row    = R_aux_to.row(1).transpose();
    _J(2, 2)          = partial_atan2_R.dot(R_row);
  }

  INSTANTIATE(SE2PriorErrorFactor)

} // namespace srrg2_solver
