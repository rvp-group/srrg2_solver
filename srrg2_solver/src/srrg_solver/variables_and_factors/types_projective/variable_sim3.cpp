#include "variable_sim3.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  void VariableSim3Base::setZero() {
    _estimate.setIdentity();
  }

  void VariableSim3Base::_drawImpl(ViewerCanvasPtr canvas_) const {
    if (!canvas_)
      throw std::runtime_error("VariableSim3Base::draw|invalid canvas");
    canvas_->pushColor();
    canvas_->setColor(srrg2_core::ColorPalette::color3fBlue());
    canvas_->pushMatrix();
    canvas_->multMatrix(_estimate.matrix());
    //      canvas_->putSphere(0.1);
    canvas_->putReferenceSystem(1);
    canvas_->popMatrix();
    canvas_->popAttribute();
  }

  void VariableSim3Base::serialize(ObjectData& odata, IdContext& context) {
    Identifiable::serialize(odata, context);
    odata.setInt("graph_id", this->graphId());
    odata.setInt("status", this->status());
    odata.setEigen<typename EstimateType::MatrixType>("estimate", this->_estimate.matrix());
  }

  void VariableSim3Base::deserialize(ObjectData& odata, IdContext& context) {
    Identifiable::deserialize(odata, context);
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
      throw std::runtime_error("Variable_::deserialize|ERROR, invalid status value in file");
    }
    this->setStatus(st);

    EstimateType est;
    est.matrix() = odata.getEigen<EstimateType::MatrixType>("estimate");
    this->setEstimate(est);
  }

}
