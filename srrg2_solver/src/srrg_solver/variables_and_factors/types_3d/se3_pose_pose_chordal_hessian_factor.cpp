#include "se3_pose_pose_chordal_hessian_factor.h"
#include <srrg_solver/solver_core/factor_impl.cpp>

namespace srrg2_solver {
  using namespace srrg2_core;
  SE3PosePoseChordalHessianFactor::SE3PosePoseChordalHessianFactor() : BaseType() {
    Rx0 << 0, 0, 0, 0, 0, -1, 0, 1, 0;
    Ry0 << 0, 0, 1, 0, 0, 0, -1, 0, 0;
    Rz0 << 0, -1, 0, 1, 0, 0, 0, 0, 0;
    _measurement.setIdentity();
    _omega.setIdentity();
    _pose_omega.setIdentity();
  }

  void SE3PosePoseChordalHessianFactor::setInformationMatrix(const Matrix6f& omega) {
    _pose_omega = omega;
    _omega.setIdentity();
    const Matrix3f& Rz = _measurement.linear();
    JacobianMatrixType Je;
    Je.setZero();
    const Matrix3f dR_x = 2.f * Rx0 * Rz;
    const Matrix3f dR_y = 2.f * Ry0 * Rz;
    const Matrix3f dR_z = 2.f * Rz0 * Rz;

    Eigen::Matrix<float, 9, 1> dr_x_flattened, dr_y_flattened, dr_z_flattened;
    dr_x_flattened << dR_x.col(0), dR_x.col(1), dR_x.col(2);
    dr_y_flattened << dR_y.col(0), dR_y.col(1), dR_y.col(2);
    dr_z_flattened << dR_z.col(0), dR_z.col(1), dR_z.col(2);

    Je.block<9, 1>(0, 3) = dr_x_flattened;
    Je.block<9, 1>(0, 4) = dr_y_flattened;
    Je.block<9, 1>(0, 5) = dr_z_flattened;
    Je.block<3, 3>(9, 0) = Matrix3f::Identity();
    auto Je_cross        = srrg2_core::pseudoInverse(Je);
    _omega        = Je_cross.transpose() * omega * Je_cross;
  }

  void SE3PosePoseChordalHessianFactor::setMeasurement(const Isometry3f& measurement_) {
    _measurement = measurement_;
    // tg recompute projected information matrix based on the new measurement
    Matrix6f omega = _pose_omega;
    setInformationMatrix(omega);
  }

  void SE3PosePoseChordalHessianFactor::compute(bool chi_only, bool force) {
    this->_stats.status = FactorStats::Status::Suppressed;
    this->_stats.chi    = 0;
    if (!this->isActive() && !force) {
      return;
    }
    const VariableSE3EulerLeft* from = _variables.at<0>();
    const VariableSE3EulerLeft* to   = _variables.at<1>();
    // tg extract quantities required for jacobian/hessian computation
    const Isometry3f& Ti = from->estimate();
    const Matrix3f& Ri   = Ti.linear();
    const Isometry3f& Tj = to->estimate();
    const Matrix3f& Rj   = Tj.linear();
    const Vector3f& tj   = Tj.translation();
    if (!isValid()) {
      return;
    }
    // tg compute matrix error
    const Isometry3f prediction = Ti.inverse() * Tj;
    Isometry3f error            = Isometry3f::Identity();
    error.matrix()              = prediction.matrix() - _measurement.matrix();
    // tg flatten
    ErrorVectorType e           = srrg2_core::geometry3d::flattenByCols(error);
    this->_stats.chi            = e.transpose() * _omega * e;
    this->robustify();
    if (chi_only) {
      return;
    }
    // tg compute jacobian
    JacobianMatrixType J;
    J.setZero();
    const Matrix3f dR_x = Ri.transpose() * Rx0 * Rj;
    const Matrix3f dR_y = Ri.transpose() * Ry0 * Rj;
    const Matrix3f dR_z = Ri.transpose() * Rz0 * Rj;

    Eigen::Matrix<float, 9, 1> dr_x_flattened, dr_y_flattened, dr_z_flattened;
    dr_x_flattened << dR_x.col(0), dR_x.col(1), dR_x.col(2);
    dr_y_flattened << dR_y.col(0), dR_y.col(1), dR_y.col(2);
    dr_z_flattened << dR_z.col(0), dR_z.col(1), dR_z.col(2);

    J.block<9, 1>(0, 3) = dr_x_flattened;
    J.block<9, 1>(0, 4) = dr_y_flattened;
    J.block<9, 1>(0, 5) = dr_z_flattened;
    J.block<3, 3>(9, 0) = Ri.transpose();
    J.block<3, 3>(9, 3) = -Ri.transpose() * geometry3d::skew(tj);
    // tg robustify H and b
    const float& w      = this->_kernel_scales[1];
    Matrix6f H          = w * J.transpose() * _omega * J;
    Vector6f b          = w * J.transpose() * _omega * e;
    // add blocks
    int idx_r = blockOffset<0, 0>();
    if (this->_H_blocks[idx_r]) {
      Eigen::Map<Matrix6f> target_Hr(this->_H_blocks[idx_r]->storage());
      Eigen::Map<Vector6f> target_br(this->_b_blocks[0]->storage());
      target_Hr.noalias() += H;
      target_br.noalias() += b;
    }
    int idx_c = blockOffset<1, 1>();
    if (this->_H_blocks[idx_c]) {
      Eigen::Map<Matrix6f> target_Hc(this->_H_blocks[idx_c]->storage());
      Eigen::Map<Vector6f> target_bc(this->_b_blocks[1]->storage());
      target_Hc.noalias() += H;
      target_bc.noalias() -= b;
    }

    int idx_rc = blockOffset<0, 1>();
    if (this->_H_blocks[idx_rc]) {
      Eigen::Map<Matrix6f> target_Hrc(this->_H_blocks[idx_rc]->storage());
      target_Hrc.noalias() -= H;
    }
  }
  void SE3PosePoseChordalHessianFactor::serialize(ObjectData& odata, IdContext& context) {
    Identifiable::serialize(odata, context);
    odata.setInt("graph_id", BaseType::graphId());
    odata.setBool("enabled", BaseType::enabled());

    ArrayData* adata = new ArrayData;
    for (int pos = 0; pos < NumVariables; ++pos) {
      adata->add((int) BaseType::variableId(pos));
    }
    odata.setField("variables", adata);

    MeasurementOwnerBase* m_owner = dynamic_cast<MeasurementOwnerBase*>(this);
    if (m_owner) {
      m_owner->serializeMeasurement(odata, context);
    }

    ArrayData* idata = new ArrayData;
    for (int r = 0; r < 6; ++r) {
      for (int c = r; c < 6; ++c) {
        idata->add(_pose_omega(r, c));
      }
    }
    odata.setField("information", idata);
  }

  void SE3PosePoseChordalHessianFactor::deserialize(ObjectData& odata, IdContext& context) {
    Identifiable::deserialize(odata, context);
    FactorBase::_graph_id = odata.getInt("graph_id");
    if (odata.getField("enabled")) {
      FactorBase::_enabled = odata.getBool("enabled");
    }
    ArrayData* adata = dynamic_cast<ArrayData*>(odata.getField("variables"));
    int pos          = 0;
    for (auto it = adata->begin(); it != adata->end(); ++it) {
      this->_variables.setGraphId(pos, (*it)->getInt());
      ++pos;
    }

    MeasurementOwnerBase* m_owner = dynamic_cast<MeasurementOwnerBase*>(this);
    if (m_owner) {
      m_owner->deserializeMeasurement(odata, context);
    }

    ArrayData* idata = dynamic_cast<ArrayData*>(odata.getField("information"));
    int k            = 0;
    for (int r = 0; r < 6; ++r) {
      for (int c = r; c < 6; ++c, ++k) {
        _pose_omega(r, c) = (*idata)[k].getFloat();
        _pose_omega(c, r) = _pose_omega(r, c);
      }
    }
    setInformationMatrix(_pose_omega);
  }
} // namespace srrg2_solver
