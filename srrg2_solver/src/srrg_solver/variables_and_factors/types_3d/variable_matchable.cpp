#include "variable_matchable.h"
#include "srrg_solver/solver_core/instance_macros.h"
#include "srrg_solver/solver_core/variable_impl.cpp"

namespace srrg2_solver {
  //! @brief object life (ds style)
  VariableMatchable::~VariableMatchable() {
    // ia nothing to do here
  }

  void VariableMatchable::setZero() {
    this->_tainted=true;
  }

  void VariableMatchable::applyPerturbation(const srrg2_core::Vector5f& pert_)  {
    this->_tainted=true;
    Matrix3f dR;
    dR = AngleAxisf(pert_[3], Vector3f::UnitY()) * AngleAxisf(pert_[4], Vector3f::UnitZ());

    _estimate.origin().noalias() += pert_.head(3);
    _estimate.rotation().noalias() = _estimate.rotation() * dR;

    // ia enforcing orthonormality
    const Matrix3f R_backup = _estimate.rotation();
    Matrix3f E              = R_backup.transpose() * R_backup;
    E.diagonal().array() -= 1;

    _estimate.rotation() -= 0.5 * R_backup * E;
  }

  //! @brief serialization of the variable through BOSS.
  void VariableMatchable::serialize(ObjectData& odata, IdContext& context)  {
    Identifiable::serialize(odata, context);
    // ia first easy things
    odata.setInt("graph_id", this->graphId());
    odata.setInt("status", this->status());

    // ia type
    odata.setInt("type", _estimate.type());

    // ia array data to serialize origin
    ArrayData* origin_data = new ArrayData;
    for (uint8_t k = 0; k < _estimate.origin().rows(); ++k) {
      origin_data->add(_estimate.origin()[k]);
    }

    // ia array data to serialize rotation matrix
    ArrayData* rotation_data = new ArrayData;
    for (uint8_t r = 0; r < _estimate.rotation().rows(); ++r) {
      for (uint8_t c = 0; c < _estimate.rotation().cols(); ++c) {
        rotation_data->add(_estimate.rotation()(r, c));
      }
    }

    // ia add fields
    odata.setField("origin", origin_data);
    odata.setField("rotation", rotation_data);
  }

  //! @brief deserialization of the variable through BOSS.
  void VariableMatchable::deserialize(ObjectData& odata, IdContext& context) {
    Identifiable::deserialize(odata, context);

    // ia first easy thing
    this->setGraphId(odata.getInt("graph_id"));

    VariableBase::Status st;
    int s = odata.getInt("status");
    switch (s) {
    case VariableBase::Status::Active:
      st = VariableBase::Status::Active;
      break;
    case VariableBase::Status::NonActive:
      st = VariableBase::Status::NonActive;
      break;
    case VariableBase::Status::Fixed:
      st = VariableBase::Status::Fixed;
      break;
    default:
      throw std::runtime_error("Invalid status value in file");
    }
    setStatus(st);

    // ia now meat - read the object type
    MatchableBase::Type m_type = (MatchableBase::Type)(odata.getInt("type"));
    if (m_type > MatchableBase::Type::Surfel) {
      throw std::runtime_error("VariableMatchable::deserialize|invalid matchable type [" +
                               std::to_string(m_type) + "]");
    }
    _estimate.setType(m_type);

    // ia now meat - read the object origin
    ArrayData* origin_data = dynamic_cast<ArrayData*>(odata.getField("origin"));
    for (uint8_t k = 0; k < _estimate.origin().rows(); ++k) {
      _estimate.origin()[k] = (*origin_data)[k].getFloat();
    }

    // ia now meat - read the object rotation
    ArrayData* rotation_data = dynamic_cast<ArrayData*>(odata.getField("rotation"));
    uint8_t i                = 0;
    for (uint8_t r = 0; r < _estimate.rotation().rows(); ++r) {
      for (uint8_t c = 0; c < _estimate.rotation().cols(); ++c, ++i) {
        _estimate.rotation()(r, c) = (*rotation_data)[i].getFloat();
      }
    }
  }

  void VariableMatchable::draw(ViewerCanvasPtr canvas_) const {
    if (!canvas_)
      throw std::runtime_error("VariableMatchable::draw|invalid canvas");
    canvas_->pushColor();
    canvas_->setColor(srrg2_core::ColorPalette::color3fOrange());
    canvas_->putPoints(1, &_estimate.origin());
    canvas_->popAttribute();
  }

  INSTANTIATE(VariableMatchable)
} // namespace srrg2_solver
