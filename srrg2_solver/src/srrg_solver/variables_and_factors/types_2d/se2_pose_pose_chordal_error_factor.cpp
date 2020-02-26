#include "se2_pose_pose_chordal_error_factor.h"


//! include this: this contains all the implementations of the factors
//! that are hidden to the modules that do not need them to avoid excessive compilation times (EVIL)
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  void SE2PosePoseChordalErrorFactor::errorAndJacobian(bool error_only_) {
    VariableSE2Right* v_from = _variables.at<0>();
    VariableSE2Right* v_to   = _variables.at<1>();

    const Isometry2f& from = v_from->estimate();
    const Isometry2f& to   = v_to->estimate();
    Isometry2f prediction;
    prediction.setIdentity();
    prediction.linear()      = from.linear().transpose() * to.linear();
    prediction.translation() = from.linear().transpose() * (to.translation() - from.translation());
    _e = flattenIsometry(prediction) - flattenIsometry(this->_measurement);
    if (error_only_) {
      return;
    }

    Matrix2f aux_from;
    aux_from << 0, 1, -1, 0;
    Matrix2f aux_to;
    aux_to << 0, -1, 1, 0;

    Matrix2f from_dtheta_from = aux_from * prediction.linear();
    Matrix2f to_dtheta_to     = prediction.linear() * aux_to;
    const Vector2f& t         = prediction.translation();

    auto J_from = jacobian<0>();
    J_from.setZero();
    J_from.block<2, 2>(4, 0) = -Matrix2f::Identity();
    J_from.block<2, 1>(4, 2) = geometry2d::skew(t);
    J_from.block<2, 1>(0, 2) = from_dtheta_from.col(0);
    J_from.block<2, 1>(2, 2) = from_dtheta_from.col(1);

    auto J_to = jacobian<1>();
    J_to.setZero();
    J_to.block<2, 2>(4, 0) = prediction.linear();
    J_to.block<2, 1>(0, 2) = to_dtheta_to.col(0);
    J_to.block<2, 1>(2, 2) = to_dtheta_to.col(1);
  }

  INSTANTIATE(SE2PosePoseChordalErrorFactor)

} // namespace srrg2_solver
