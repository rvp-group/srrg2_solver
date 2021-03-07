#include "se3_pose_matchable_error_factor.h"

//! include this: this contains all the implementations of the factors
//! that are hidden to the modules that do not need them to avoid excessive compilation times (EVIL)
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {

  void SE3PoseMatchableEulerLeftErrorFactor::errorAndJacobian(bool error_only_) {
    using Scalar              = MeasurementType::Scalar;
    using Matrix3             = srrg2_core::Matrix3_<Scalar>;
    using Vector3             = srrg2_core::Vector3_<Scalar>;
    using Vector2             = srrg2_core::Vector2_<Scalar>;
    using Matrix3_2           = Eigen::Matrix<Scalar, 3, 2>;
    using JacobianIMatrixType = Eigen::Matrix<Scalar, 7, 6>;
    using JacobianJMatrixType = Eigen::Matrix<Scalar, 7, 5>;

    // ia error and jacobian
    VariableSE3EulerLeft* v_from = _variables.at<0>();
    VariableMatchable* v_to      = _variables.at<1>();

    const auto& pose = v_from->estimate();
    const Vector3& t = pose.translation();
    const Matrix3& R = pose.linear();

    const Vector3& pl = v_to->estimate().origin();
    const Matrix3& Rl = v_to->estimate().rotation();

    const Vector3& pz = _measurement.origin();
    const Matrix3& Rz = _measurement.rotation();

    const Vector3 ep = Rl.transpose() * (R * pz + t - pl);
    const Vector3 ed = R * Rz.col(0) - Rl.col(0);
    const Scalar eo  = (R * Rz).col(0).transpose() * Rl.col(0);

    _e.block<3, 1>(0, 0) = ep;
    _e.block<3, 1>(3, 0) = ed;
    _e[6]                = eo;

    if (error_only_) {
      return;
    }

    JacobianIMatrixType Ji = JacobianIMatrixType::Zero();
    JacobianJMatrixType Jj = JacobianJMatrixType::Zero();
    // ia compute Ji - components
    Vector3 v      = R * pz + t;
    Matrix3 dep_dt = Rl.transpose();
    Matrix3 dep_dR = -Rl.transpose() * srrg2_core::geometry3d::skew(v);

    // Matrix3 ded_dt = Matrix3::Zero();
    v              = (R * Rz).col(0);
    Matrix3 ded_dR = -srrg2_core::geometry3d::skew(v);

    // Vector3 deo_dt = Vector3::Zero();
    v                     = Rl.col(0);
    Matrix3 deo_dR_matrix = Rz.transpose() * R.transpose() * srrg2_core::geometry3d::skew(v);
    Vector3 deo_dR        = deo_dR_matrix.row(0);

    Ji.block<3, 3>(0, 0) = dep_dt;
    Ji.block<3, 3>(0, 3) = dep_dR;
    Ji.block<3, 3>(3, 3) = ded_dR;
    Ji.block<1, 3>(6, 3) = deo_dR.transpose();

    // ia compute Jj - components
    v                 = Rl.transpose() * (R * pz + t - pl);
    Matrix3 dep_dpl   = -Rl.transpose();
    Matrix3_2 dep_dRl = srrg2_core::geometry3d::skew(v).block<3, 2>(0, 1);

    // Matrix3 ded_dpl = Matrix3::Zero();
    v                 = Vector3::UnitX();
    Matrix3_2 ded_dRl = Rl * srrg2_core::geometry3d::skew(v).block<3, 2>(0, 1);

    // Vector3 deo_dpl = Vector3::Zero();
    v = Vector3::UnitX();
    Matrix3_2 deo_dRl_matrix =
      Rz.transpose() * R.transpose() * Rl * srrg2_core::geometry3d::skew(v).block<3, 2>(0, 1);
    Vector2 deo_dRl = -deo_dRl_matrix.row(0);

    Jj.block<3, 3>(0, 0) = dep_dpl;
    Jj.block<3, 2>(0, 3) = dep_dRl;
    Jj.block<3, 2>(3, 3) = ded_dRl;
    Jj.block<1, 2>(6, 3) = deo_dRl.transpose();

    jacobian<0>() = Ji;
    jacobian<1>() = Jj;
  }
  //! @brief serializes the factor
  void SE3PoseMatchableEulerLeftErrorFactor::serializeMeasurement(ObjectData& odata, IdContext& context) {

    // ia array data to serialize origin
    ArrayData* origin_data = new ArrayData;
    for (uint8_t k = 0; k < _measurement.origin().rows(); ++k) {
      origin_data->add(_measurement.origin()[k]);
    }

    // ia array data to serialize rotation matrix
    ArrayData* rotation_data = new ArrayData;
    for (uint8_t r = 0; r < _measurement.rotation().rows(); ++r) {
      for (uint8_t c = 0; c < _measurement.rotation().cols(); ++c) {
        rotation_data->add(_measurement.rotation()(r, c));
      }
    }

    // ia write data
    odata.setInt("type", _measurement.type());
    odata.setField("origin", origin_data);
    odata.setField("rotation", rotation_data);
  }

  //! @brief deserializes the factor
  void SE3PoseMatchableEulerLeftErrorFactor::deserializeMeasurement(ObjectData& odata, IdContext& context) {
    // ia now meat - read the object type
    MatchableBase::Type m_type = (MatchableBase::Type)(odata.getInt("type"));
    if (m_type > MatchableBase::Type::Surfel) {
      throw std::runtime_error("VariableMatchable::deserialize|invalid matchable type [" +
                               std::to_string(m_type) + "]");
    }
    _measurement.setType(m_type);

    // ia now meat - read the object origin
    ArrayData* origin_data = dynamic_cast<ArrayData*>(odata.getField("origin"));
    for (uint8_t k = 0; k < _measurement.origin().rows(); ++k) {
      _measurement.origin()[k] = (*origin_data)[k].getFloat();
    }

    // ia now meat - read the object rotation
    ArrayData* rotation_data = dynamic_cast<ArrayData*>(odata.getField("rotation"));
    uint8_t i                = 0;
    for (uint8_t r = 0; r < _measurement.rotation().rows(); ++r) {
      for (uint8_t c = 0; c < _measurement.rotation().cols(); ++c, ++i) {
        _measurement.rotation()(r, c) = (*rotation_data)[i].getFloat();
      }
    }
  }

  void SE3PoseMatchableEulerLeftErrorFactor::_drawImpl(ViewerCanvasPtr canvas_) const {
    if (!canvas_) {
      throw std::runtime_error("SE3PoseMatchableErrorFactor::draw|invalid canvas");
    }
    Vector3f coords[2];
    coords[0] = static_cast<const VariableSE3EulerLeft*>(variable(0))->estimate().translation();
    coords[1] = static_cast<const VariableMatchable*>(variable(1))->estimate().origin();
    canvas_->pushColor();
    canvas_->setColor(srrg2_core::ColorPalette::color3fOrange());
    canvas_->putLine(2, coords);
    canvas_->popAttribute();
  }

  INSTANTIATE(SE3PoseMatchableEulerLeftErrorFactor)

} // namespace srrg2_solver
