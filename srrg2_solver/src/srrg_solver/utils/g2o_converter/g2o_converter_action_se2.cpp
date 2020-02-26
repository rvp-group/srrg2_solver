#include "g2o_converter_action_se2.h"
#include "g2o_converter.h"

namespace srrg2_solver {

  void ConverterActionVariableSE2RightAD::readFromG2O(std::istream& stream_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariableSE2RightAD::readFromG2O|invalid graph pointer");

    BossType::EstimateType estimate = BossType::EstimateType::Identity();

    // ia read the ID
    int graph_id = 0;
    stream_ >> graph_id;

    estimate = readG2OIsometry2<Scalar>(stream_);

    _boss_object = new BossType();
    _boss_object->setGraphId(graph_id);
    _boss_object->setEstimate(estimate);
    _graph_ptr->addVariable(VariableBasePtr(_boss_object));
  }

  void ConverterActionVariableSE2RightAD::writeFromBoss(std::ofstream& stream_, void* object_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariableSE2RightAD::writeFromBoss|invalid graph pointer");

    _boss_object = reinterpret_cast<BossType*>(object_);
    assert(_boss_object != nullptr &&
           "ConverterActionVariableSE2RightAD::writeFromBoss|invalid boss object");

    stream_ << _boss_object->graphId() << " ";
    writeG2OIsometry2(stream_, _boss_object->estimate());

    if (_boss_object->status() == VariableBase::Status::Fixed) {
      stream_ << "\nFIX " << _boss_object->graphId();
    }
  }

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  void ConverterActionVariableSE2Right::readFromG2O(std::istream& stream_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariableSE2Right::readFromG2O|invalid graph pointer");

    BossType::EstimateType estimate = BossType::EstimateType::Identity();

    // ia read the ID
    int graph_id = 0;
    stream_ >> graph_id;

    estimate = readG2OIsometry2<Scalar>(stream_);

    _boss_object = new BossType();
    _boss_object->setGraphId(graph_id);
    _boss_object->setEstimate(estimate);
    _graph_ptr->addVariable(VariableBasePtr(_boss_object));
  }

  void ConverterActionVariableSE2Right::writeFromBoss(std::ofstream& stream_, void* object_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariableSE2Right::writeFromBoss|invalid graph pointer");

    _boss_object = reinterpret_cast<BossType*>(object_);
    assert(_boss_object != nullptr &&
           "ConverterActionVariableSE2Right::writeFromBoss|invalid boss object");

    stream_ << _boss_object->graphId() << " ";
    writeG2OIsometry2(stream_, _boss_object->estimate());

    if (_boss_object->status() == VariableBase::Status::Fixed) {
      stream_ << "\nFIX " << _boss_object->graphId();
    }
  }
  void ConverterActionVariableSE2LeftAD::readFromG2O(std::istream& stream_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariableSE2LeftAD::readFromG2O|invalid graph pointer");

    BossType::EstimateType estimate = BossType::EstimateType::Identity();

    // ia read the ID
    int graph_id = 0;
    stream_ >> graph_id;

    estimate = readG2OIsometry2<Scalar>(stream_);

    _boss_object = new BossType();
    _boss_object->setGraphId(graph_id);
    _boss_object->setEstimate(estimate);
    _graph_ptr->addVariable(VariableBasePtr(_boss_object));
  }

  void ConverterActionVariableSE2LeftAD::writeFromBoss(std::ofstream& stream_, void* object_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariableSE2LeftAD::writeFromBoss|invalid graph pointer");

    _boss_object = reinterpret_cast<BossType*>(object_);
    assert(_boss_object != nullptr &&
           "ConverterActionVariableSE2LeftAD::writeFromBoss|invalid boss object");

    stream_ << _boss_object->graphId() << " ";
    writeG2OIsometry2(stream_, _boss_object->estimate());

    if (_boss_object->status() == VariableBase::Status::Fixed) {
      stream_ << "\nFIX " << _boss_object->graphId();
    }
  }

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //

  void ConverterActionVariableSE2Left::readFromG2O(std::istream& stream_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariableSE2Left::readFromG2O|invalid graph pointer");

    BossType::EstimateType estimate = BossType::EstimateType::Identity();

    // ia read the ID
    int graph_id = 0;
    stream_ >> graph_id;

    estimate = readG2OIsometry2<Scalar>(stream_);

    _boss_object = new BossType();
    _boss_object->setGraphId(graph_id);
    _boss_object->setEstimate(estimate);
    _graph_ptr->addVariable(VariableBasePtr(_boss_object));
  }

  void ConverterActionVariableSE2Left::writeFromBoss(std::ofstream& stream_, void* object_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariableSE2Left::writeFromBoss|invalid graph pointer");

    _boss_object = reinterpret_cast<BossType*>(object_);
    assert(_boss_object != nullptr &&
           "ConverterActionVariableSE2Left::writeFromBoss|invalid boss object");

    stream_ << _boss_object->graphId() << " ";
    writeG2OIsometry2(stream_, _boss_object->estimate());

    if (_boss_object->status() == VariableBase::Status::Fixed) {
      stream_ << "\nFIX " << _boss_object->graphId();
    }
  }

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  void ConverterActionVariablePoint2AD::readFromG2O(std::istream& stream_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariablePoint2AD::readFromG2O|invalid graph pointer");

    BossType::EstimateType estimate = BossType::EstimateType::Zero();

    // ia read the ID
    int graph_id = 0;
    stream_ >> graph_id;

    stream_ >> estimate.x() >> estimate.y();

    _boss_object = new BossType();
    _boss_object->setGraphId(graph_id);
    _boss_object->setEstimate(estimate);
    _graph_ptr->addVariable(VariableBasePtr(_boss_object));
  }

  void ConverterActionVariablePoint2AD::writeFromBoss(std::ofstream& stream_, void* object_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariablePoint2AD::writeFromBoss|invalid graph pointer");

    _boss_object = reinterpret_cast<BossType*>(object_);
    assert(_boss_object != nullptr &&
           "ConverterActionVariablePoint2AD::writeFromBoss|invalid boss object");

    stream_ << _boss_object->graphId() << " ";
    stream_ << _boss_object->estimate().x() << " " << _boss_object->estimate().y() << " ";

    if (_boss_object->status() == VariableBase::Status::Fixed) {
      stream_ << "\nFIX " << _boss_object->graphId();
    }
  }

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  void ConverterActionVariablePoint2::readFromG2O(std::istream& stream_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariablePoint2::readFromG2O|invalid graph pointer");

    BossType::EstimateType estimate = BossType::EstimateType::Zero();

    // ia read the ID
    int graph_id = 0;
    stream_ >> graph_id;

    stream_ >> estimate.x() >> estimate.y();

    _boss_object = new BossType();
    _boss_object->setGraphId(graph_id);
    _boss_object->setEstimate(estimate);
    _graph_ptr->addVariable(VariableBasePtr(_boss_object));
  }

  void ConverterActionVariablePoint2::writeFromBoss(std::ofstream& stream_, void* object_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariablePoint2::writeFromBoss|invalid graph pointer");

    _boss_object = reinterpret_cast<BossType*>(object_);
    assert(_boss_object != nullptr &&
           "ConverterActionVariablePoint2::writeFromBoss|invalid boss object");

    stream_ << _boss_object->graphId() << " ";
    stream_ << _boss_object->estimate().x() << " " << _boss_object->estimate().y() << " ";

    if (_boss_object->status() == VariableBase::Status::Fixed) {
      stream_ << "\nFIX " << _boss_object->graphId();
    }
  }

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  void ConverterActionSE2PosePoseGeodesicErrorFactor::readFromG2O(std::istream& stream_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionSE2PosePoseGeodesicFactor::readFromG2O|invalid graph pointer");

    BossType::MeasurementType measurement = BossType::MeasurementType::Identity();
    BossType::InformationMatrixType omega = BossType::InformationMatrixType::Identity();

    // ia read the ID
    int graph_id_0 = -1;
    int graph_id_1 = -1;
    stream_ >> graph_id_0 >> graph_id_1;

    // ia read the measurement
    measurement = readG2OIsometry2<Scalar>(stream_);

    // ia read the omega
    for (int i = 0; i < omega.rows() && stream_.good(); i++) {
      for (int j = i; j < omega.cols() && stream_.good(); j++) {
        stream_ >> omega(i, j);
        if (i != j)
          omega(j, i) = omega(i, j);
      }
    }

    _boss_object = new BossType();
    _boss_object->setMeasurement(measurement);
    _boss_object->setInformationMatrix(omega);
    _boss_object->setVariableId(0, graph_id_0);
    _boss_object->setVariableId(1, graph_id_1);
    _graph_ptr->addFactor(FactorBasePtr(_boss_object));
  }

  void ConverterActionSE2PosePoseGeodesicErrorFactor::writeFromBoss(std::ofstream& stream_,
                                                                    void* object_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionSE2PosePoseGeodesicFactor::writeFromBoss|invalid graph pointer");

    _boss_object = reinterpret_cast<BossType*>(object_);
    assert(_boss_object != nullptr &&
           "ConverterActionSE2PosePoseGeodesicFactor::writeFromBoss|invalid boss object");

    stream_ << _boss_object->variableId(0) << " " << _boss_object->variableId(1) << " ";
    writeG2OIsometry2(stream_, _boss_object->measurement());

    const auto& omega = _boss_object->informationMatrix();
    for (int i = 0; i < omega.rows(); i++) {
      for (int j = i; j < omega.cols(); j++) {
        stream_ << omega(i, j) << " ";
      }
    }
  }

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  void ConverterActionSE2PosePointErrorFactor::readFromG2O(std::istream& stream_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionSE2PosePointFactor::readFromG2O|invalid graph pointer");

    BossType::MeasurementType measurement = BossType::MeasurementType::Zero();
    BossType::InformationMatrixType omega = BossType::InformationMatrixType::Identity();

    // ia read the ID
    int graph_id_0 = -1;
    int graph_id_1 = -1;
    stream_ >> graph_id_0 >> graph_id_1;

    // ia read the measurement
    stream_ >> measurement.x() >> measurement.y();

    // ia read the omega
    for (int i = 0; i < omega.rows() && stream_.good(); i++) {
      for (int j = i; j < omega.cols() && stream_.good(); j++) {
        stream_ >> omega(i, j);
        if (i != j)
          omega(j, i) = omega(i, j);
      }
    }

    _boss_object = new BossType();
    _boss_object->setMeasurement(measurement);
    _boss_object->setInformationMatrix(omega);
    _boss_object->setVariableId(0, graph_id_0);
    _boss_object->setVariableId(1, graph_id_1);
    _graph_ptr->addFactor(FactorBasePtr(_boss_object));
  }

  void ConverterActionSE2PosePointErrorFactor::writeFromBoss(std::ofstream& stream_,
                                                             void* object_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionSE2PosePointFactor::writeFromBoss|invalid graph pointer");

    _boss_object = reinterpret_cast<BossType*>(object_);
    assert(_boss_object != nullptr &&
           "ConverterActionSE2PosePointFactor::writeFromBoss|invalid boss object");

    stream_ << _boss_object->variableId(0) << " " << _boss_object->variableId(1) << " ";

    // ia write the measurement
    stream_ << _boss_object->measurement().x() << " " << _boss_object->measurement().y() << " ";

    // ia write information
    const auto& omega = _boss_object->informationMatrix();
    for (int i = 0; i < omega.rows(); i++) {
      for (int j = i; j < omega.cols(); j++) {
        stream_ << omega(i, j) << " ";
      }
    }
  }

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  void ConverterActionSE2PosePointBearingErrorFactor::readFromG2O(std::istream& stream_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionSE2PosePointBearingFactor::readFromG2O|invalid graph pointer");

    BossType::MeasurementType measurement = BossType::MeasurementType::Zero();
    BossType::InformationMatrixType omega = BossType::InformationMatrixType::Identity();

    // ia read the ID
    int graph_id_0 = -1;
    int graph_id_1 = -1;
    stream_ >> graph_id_0 >> graph_id_1;

    // ia read the measurement
    stream_ >> measurement.x();

    // ia read the omega
    for (int i = 0; i < omega.rows() && stream_.good(); i++) {
      for (int j = i; j < omega.cols() && stream_.good(); j++) {
        stream_ >> omega(i, j);
        if (i != j)
          omega(j, i) = omega(i, j);
      }
    }

    _boss_object = new BossType();
    _boss_object->setMeasurement(measurement);
    _boss_object->setInformationMatrix(omega);
    _boss_object->setVariableId(0, graph_id_0);
    _boss_object->setVariableId(1, graph_id_1);
    _graph_ptr->addFactor(FactorBasePtr(_boss_object));
  }

  void ConverterActionSE2PosePointBearingErrorFactor::writeFromBoss(std::ofstream& stream_,
                                                                    void* object_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionSE2PosePointBearingFactor::writeFromBoss|invalid graph pointer");

    _boss_object = reinterpret_cast<BossType*>(object_);
    assert(_boss_object != nullptr &&
           "ConverterActionSE2PosePointBearingFactor::writeFromBoss|invalid boss object");

    stream_ << _boss_object->variableId(0) << " " << _boss_object->variableId(1) << " ";

    // ia write the measurement
    stream_ << _boss_object->measurement().x() << " ";

    // ia write information
    const auto& omega = _boss_object->informationMatrix();
    for (int i = 0; i < omega.rows(); i++) {
      for (int j = i; j < omega.cols(); j++) {
        stream_ << omega(i, j) << " ";
      }
    }
  }
} // namespace srrg2_solver