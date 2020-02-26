#include "g2o_converter_action_se3.h"
#include "g2o_converter.h"

namespace srrg2_solver {

  void ConverterActionVariableSE3QuaternionRight::readFromG2O(std::istream& stream_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariableSE3QuaternionRight::readFromG2O|invalid graph pointer");

    BossType::EstimateType estimate = BossType::EstimateType::Identity();

    // ia read the ID
    int graph_id = 0;
    stream_ >> graph_id;

    estimate = readG2OIsometry3<Scalar>(stream_);

    _boss_object = new BossType();
    _boss_object->setGraphId(graph_id);
    _boss_object->setEstimate(estimate);
    _graph_ptr->addVariable(VariableBasePtr(_boss_object));
  }

  void ConverterActionVariableSE3QuaternionRight::writeFromBoss(std::ofstream& stream_,
                                                                void* object_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariableSE3QuaternionRight::writeFromBoss|invalid graph pointer");

    _boss_object = reinterpret_cast<BossType*>(object_);
    assert(_boss_object != nullptr &&
           "ConverterActionVariableSE3QuaternionRight::writeFromBoss|invalid boss object");

    stream_ << _boss_object->graphId() << " ";
    writeG2OIsometry3(stream_, _boss_object->estimate());

    if (_boss_object->status() == VariableBase::Status::Fixed) {
      stream_ << "\nFIX " << _boss_object->graphId();
    }
  }

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  void ConverterActionVariableSE3QuaternionRightAD::readFromG2O(std::istream& stream_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariableSE3QuaternionRightAD::readFromG2O|invalid graph pointer");
    BossType::EstimateType estimate = BossType::EstimateType::Identity();

    // ia read the ID
    int graph_id = 0;
    stream_ >> graph_id;

    // ia read the estimate
    estimate = readG2OIsometry3<Scalar>(stream_);

    _boss_object = new BossType();
    _boss_object->setGraphId(graph_id);
    _boss_object->setEstimate(estimate);
    _graph_ptr->addVariable(VariableBasePtr(_boss_object));
  }

  void ConverterActionVariableSE3QuaternionRightAD::writeFromBoss(std::ofstream& stream_,
                                                                  void* object_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariableSE3QuaternionRightAD::writeFromBoss|invalid graph pointer");

    _boss_object = reinterpret_cast<BossType*>(object_);
    assert(_boss_object != nullptr &&
           "ConverterActionVariableSE3QuaternionRightAD::writeFromBoss|invalid boss object");

    stream_ << _boss_object->graphId() << " ";
    writeG2OIsometry3(stream_, _boss_object->estimate());

    if (_boss_object->status() == VariableBase::Status::Fixed) {
      stream_ << "\nFIX " << _boss_object->graphId();
    }
  }

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  void ConverterActionVariableSE3EulerRight::readFromG2O(std::istream& stream_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVertexSE3EulerRight::readFromG2O|invalid graph pointer");

    BossType::EstimateType estimate = BossType::EstimateType::Identity();

    // ia read the ID
    int graph_id = 0;
    stream_ >> graph_id;

    estimate = readG2OIsometry3<Scalar>(stream_);

    _boss_object = new BossType();
    _boss_object->setGraphId(graph_id);
    _boss_object->setEstimate(estimate);
    _graph_ptr->addVariable(VariableBasePtr(_boss_object));
  }

  void ConverterActionVariableSE3EulerRight::writeFromBoss(std::ofstream& stream_, void* object_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariableSE3EulerRight::writeFromBoss|invalid graph pointer");

    _boss_object = reinterpret_cast<BossType*>(object_);
    assert(_boss_object != nullptr &&
           "ConverterActionVariableSE3EulerRight::writeFromBoss|invalid boss object");

    stream_ << _boss_object->graphId() << " ";
    writeG2OIsometry3(stream_, _boss_object->estimate());

    if (_boss_object->status() == VariableBase::Status::Fixed) {
      stream_ << "\nFIX " << _boss_object->graphId();
    }
  }

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  void ConverterActionVariableSE3EulerRightAD::readFromG2O(std::istream& stream_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariableSE3EulerRightAD::readFromG2O|invalid graph pointer");
    BossType::EstimateType estimate = BossType::EstimateType::Identity();

    // ia read the ID
    int graph_id = 0;
    stream_ >> graph_id;

    // ia read the estimate
    estimate = readG2OIsometry3<Scalar>(stream_);

    _boss_object = new BossType();
    _boss_object->setGraphId(graph_id);
    _boss_object->setEstimate(estimate);
    _graph_ptr->addVariable(VariableBasePtr(_boss_object));
  }

  void ConverterActionVariableSE3EulerRightAD::writeFromBoss(std::ofstream& stream_,
                                                             void* object_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariableSE3EulerRightAD::writeFromBoss|invalid graph pointer");

    _boss_object = reinterpret_cast<BossType*>(object_);
    assert(_boss_object != nullptr &&
           "ConverterActionVariableSE3EulerRightAD::writeFromBoss|invalid boss object");

    stream_ << _boss_object->graphId() << " ";
    writeG2OIsometry3(stream_, _boss_object->estimate());

    if (_boss_object->status() == VariableBase::Status::Fixed) {
      stream_ << "\nFIX " << _boss_object->graphId();
    }
  }

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  void ConverterActionVariableSE3EulerLeft::readFromG2O(std::istream& stream_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariableSE3EulerLeft::readFromG2O|invalid graph pointer");

    BossType::EstimateType estimate = BossType::EstimateType::Identity();

    // ia read the ID
    int graph_id = 0;
    stream_ >> graph_id;

    estimate = readG2OIsometry3<Scalar>(stream_);

    _boss_object = new BossType();
    _boss_object->setGraphId(graph_id);
    _boss_object->setEstimate(estimate);
    _graph_ptr->addVariable(VariableBasePtr(_boss_object));
  }

  void ConverterActionVariableSE3EulerLeft::writeFromBoss(std::ofstream& stream_, void* object_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariableSE3EulerLeft::writeFromBoss|invalid graph pointer");

    _boss_object = reinterpret_cast<BossType*>(object_);
    assert(_boss_object != nullptr &&
           "ConverterActionVariableSE3EulerLeft::writeFromBoss|invalid boss object");

    stream_ << _boss_object->graphId() << " ";
    writeG2OIsometry3(stream_, _boss_object->estimate());

    if (_boss_object->status() == VariableBase::Status::Fixed) {
      stream_ << "\nFIX " << _boss_object->graphId();
    }
  }

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  void ConverterActionVariableSE3EulerLeftAD::readFromG2O(std::istream& stream_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariableSE3EulerLeftAD::readFromG2O|invalid graph pointer");

    BossType::EstimateType estimate = BossType::EstimateType::Identity();

    // ia read the ID
    int graph_id = 0;
    stream_ >> graph_id;

    estimate = readG2OIsometry3<Scalar>(stream_);

    _boss_object = new BossType();
    _boss_object->setGraphId(graph_id);
    _boss_object->setEstimate(estimate);
    _graph_ptr->addVariable(VariableBasePtr(_boss_object));
  }

  void ConverterActionVariableSE3EulerLeftAD::writeFromBoss(std::ofstream& stream_, void* object_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariableSE3EulerLeftAD::writeFromBoss|invalid graph pointer");

    _boss_object = reinterpret_cast<BossType*>(object_);
    assert(_boss_object != nullptr &&
           "ConverterActionVariableSE3EulerLeftAD::writeFromBoss|invalid boss object");

    stream_ << _boss_object->graphId() << " ";
    writeG2OIsometry3(stream_, _boss_object->estimate());

    if (_boss_object->status() == VariableBase::Status::Fixed) {
      stream_ << "\nFIX " << _boss_object->graphId();
    }
  }

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  void ConverterActionSE3PoseOffset::readFromG2O(std::istream& stream_) {
    // ia read the parameter as a normal variable
    assert(_graph_ptr != nullptr &&
           "ConverterActionSE3PoseOffset::readFromG2O|invalid graph pointer");

    BossType::EstimateType estimate = BossType::EstimateType::Identity();

    // ia read the ID
    int graph_id = 0;
    stream_ >> graph_id;

    estimate = readG2OIsometry3<BossType::EstimateType::Scalar>(stream_);

    _boss_object = new BossType();
    _boss_object->setEstimate(estimate);

    // ia now change status and the graph id
    _boss_object->setStatus(VariableBase::Status::Fixed);
    const size_t boss_parameter_id = _converter_ptr->registerG2OParameterId(graph_id);
    _boss_object->setGraphId(boss_parameter_id);
    _graph_ptr->addVariable(VariableBasePtr(_boss_object));
  }

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  void ConverterActionSE3PoseOffsetAD::readFromG2O(std::istream& stream_) {
    // ia read the parameter as a normal variable
    assert(_graph_ptr != nullptr &&
           "ConverterActionSE3PoseOffset::readFromG2O|invalid graph pointer");

    BossType::EstimateType estimate = BossType::EstimateType::Identity();

    // ia read the ID
    int graph_id = 0;
    stream_ >> graph_id;

    estimate = readG2OIsometry3<BossType::EstimateType::Scalar>(stream_);

    _boss_object = new BossType();
    _boss_object->setEstimate(estimate);

    // ia now change status and the graph id
    _boss_object->setStatus(VariableBase::Status::Fixed);
    const size_t boss_parameter_id = _converter_ptr->registerG2OParameterId(graph_id);

    std::cerr << "ConverterActionSE3PoseOffsetAD::readFromG2O|read id = " << graph_id
              << "\t converted id = " << boss_parameter_id << std::endl;
    _boss_object->setGraphId(boss_parameter_id);
    _graph_ptr->addVariable(VariableBasePtr(_boss_object));
  }

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  void ConverterActionSE3PosePoseGeodesicErrorFactor::readFromG2O(std::istream& stream_) {
    assert(_graph_ptr != nullptr && "ConverterActionSE3PosePoseGeodesicEulerErrorFactorAD::"
                                    "readFromG2O|invalid graph pointer");

    BossType::MeasurementType measurement = BossType::MeasurementType::Identity();
    BossType::InformationMatrixType omega = BossType::InformationMatrixType::Identity();

    // ia read the ID
    int graph_id_0 = -1;
    int graph_id_1 = -1;
    stream_ >> graph_id_0 >> graph_id_1;

    // ia read the measurement
    measurement = readG2OIsometry3<Scalar>(stream_);

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

  void
  ConverterActionSE3PosePoseGeodesicErrorFactor::writeFromBoss(std::ofstream& stream_,
                                                                         void* object_) {
    assert(_graph_ptr != nullptr && "ConverterActionSE3PosePoseGeodesicErrorFactor::"
                                    "writeFromBoss|invalid graph pointer");

    _boss_object = reinterpret_cast<BossType*>(object_);
    assert(_boss_object != nullptr && "ConverterActionSE3PosePoseGeodesicErrorFactor::"
                                      "writeFromBoss|invalid boss object");

    stream_ << _boss_object->variableId(0) << " " << _boss_object->variableId(1) << " ";
    writeG2OIsometry3(stream_, _boss_object->measurement());

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
  void ConverterActionSE3PosePoseChordalEulerLeftErrorFactor::readFromG2O(std::istream& stream_) {
    assert(_graph_ptr != nullptr && "ConverterActionSE3PosePoseChordalEulerLeftErrorFactor::"
                                    "readFromG2O|invalid graph pointer");

    BossType::MeasurementType measurement = BossType::MeasurementType::Identity();
    BossType::InformationMatrixType omega = BossType::InformationMatrixType::Identity();

    // ia read the ID
    int graph_id_0 = -1;
    int graph_id_1 = -1;
    stream_ >> graph_id_0 >> graph_id_1;

    // ia read the measurement
    measurement = readG2OIsometry3<Scalar>(stream_);

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

  void ConverterActionSE3PosePoseChordalEulerLeftErrorFactor::writeFromBoss(std::ofstream& stream_,
                                                                        void* object_) {
    assert(_graph_ptr != nullptr && "ConverterActionSE3PosePoseChordalEulerLeftErrorFactor::"
                                    "writeFromBoss|invalid graph pointer");

    _boss_object = reinterpret_cast<BossType*>(object_);
    assert(_boss_object != nullptr && "ConverterActionSE3PosePoseChordalEulerLeftErrorFactor::"
                                      "writeFromBoss|invalid boss object");

    stream_ << _boss_object->variableId(0) << " " << _boss_object->variableId(1) << " ";
    writeG2OIsometry3(stream_, _boss_object->measurement());

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
  void ConverterActionSE3PosePointOffsetErrorFactor::readFromG2O(std::istream& stream_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionSE3PosePointOffsetErrorFactor::readFromG2O|invalid graph pointer");

    BossType::MeasurementType measurement = BossType::MeasurementType::Zero();
    BossType::InformationMatrixType omega = BossType::InformationMatrixType::Identity();

    // ia read the ID
    int graph_id_0        = -1;
    int graph_id_1        = -1;
    int graph_id_offset_v = -1;
    stream_ >> graph_id_0 >> graph_id_1 >> graph_id_offset_v;

    assert(graph_id_0 >= 0 &&
           "ConverterActionSE3PosePointOffsetErrorFactor::readFromG2O|invalid graph_id_0");
    assert(graph_id_1 >= 0 &&
           "ConverterActionSE3PosePointOffsetErrorFactor::readFromG2O|invalid graph_id_1");
    assert(graph_id_offset_v >= 0 &&
           "ConverterActionSE3PosePointOffsetErrorFactor::readFromG2O|invalid graph_id_offset_v");

    // ia get the boss offset graph id
    const size_t& boss_graph_id_offset_v = _converter_ptr->getBossParameterId(graph_id_offset_v);

    // ia read the measurement
    for (size_t i = 0; i < 3; i++) {
      stream_ >> measurement[i];
    }

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
    _boss_object->setVariableId(2, boss_graph_id_offset_v);
    _graph_ptr->addFactor(FactorBasePtr(_boss_object));
  }

  void ConverterActionSE3PosePointOffsetErrorFactor::writeFromBoss(std::ofstream& stream_,
                                                                     void* object_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionSE3PosePointOffsetErrorFactor::writeFromBoss|invalid graph pointer");

    _boss_object = reinterpret_cast<BossType*>(object_);
    assert(_boss_object != nullptr &&
           "ConverterActionSE3PosePointOffsetErrorFactor::writeFromBoss|invalid boss object");

    stream_ << _boss_object->variableId(0) << " " << _boss_object->variableId(1) << " "
            << _boss_object->variableId(2) << " ";

    for (int i = 0; i < _boss_object->measurement().rows(); ++i) {
      stream_ << _boss_object->measurement()[i] << " ";
    }

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
  void ConverterActionVariablePoint3AD::readFromG2O(std::istream& stream_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariablePoint3AD::readFromG2O|invalid graph pointer");
    BossType::EstimateType estimate = BossType::EstimateType::Zero();

    // ia read the ID
    int graph_id = 0;
    stream_ >> graph_id;

    // ia read the estimate
    for (size_t i = 0; i < 3; i++) {
      stream_ >> estimate[i];
    }

    _boss_object = new BossType();
    _boss_object->setGraphId(graph_id);
    _boss_object->setEstimate(estimate);
    _graph_ptr->addVariable(VariableBasePtr(_boss_object));
  }

  void ConverterActionVariablePoint3AD::writeFromBoss(std::ofstream& stream_, void* object_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariablePoint3AD::writeFromBoss|invalid graph pointer");

    _boss_object = reinterpret_cast<BossType*>(object_);
    assert(_boss_object != nullptr &&
           "ConverterActionVariablePoint3AD::writeFromBoss|invalid boss object");

    stream_ << _boss_object->graphId() << " ";
    for (int i = 0; i < _boss_object->estimate().rows(); ++i) {
      stream_ << _boss_object->estimate()[i] << " ";
    }

    if (_boss_object->status() == VariableBase::Status::Fixed) {
      stream_ << "\nFIX " << _boss_object->graphId();
    }
  }

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  void ConverterActionVariablePoint3::readFromG2O(std::istream& stream_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariablePoint3::readFromG2O|invalid graph pointer");
    BossType::EstimateType estimate = BossType::EstimateType::Zero();

    // ia read the ID
    int graph_id = 0;
    stream_ >> graph_id;

    // ia read the estimate
    for (size_t i = 0; i < 3; i++) {
      stream_ >> estimate[i];
    }

    _boss_object = new BossType();
    _boss_object->setGraphId(graph_id);
    _boss_object->setEstimate(estimate);
    _graph_ptr->addVariable(VariableBasePtr(_boss_object));
  }

  void ConverterActionVariablePoint3::writeFromBoss(std::ofstream& stream_, void* object_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariablePoint3::writeFromBoss|invalid graph pointer");

    _boss_object = reinterpret_cast<BossType*>(object_);
    assert(_boss_object != nullptr &&
           "ConverterActionVariablePoint3::writeFromBoss|invalid boss object");

    stream_ << _boss_object->graphId() << " ";
    for (int i = 0; i < _boss_object->estimate().rows(); ++i) {
      stream_ << _boss_object->estimate()[i] << " ";
    }

    if (_boss_object->status() == VariableBase::Status::Fixed) {
      stream_ << "\nFIX " << _boss_object->graphId();
    }
  }

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  void ConverterActionVariableMatchable::readFromG2O(std::istream& stream_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariableMatchable::readFromG2O|invalid graph pointer");

    // ia read the ID
    int graph_id = 0;
    stream_ >> graph_id;

    // ia read the estimate
    int type                                    = -1;
    BossType::EstimateType::VectorType origin   = BossType::EstimateType::VectorType::Zero();
    BossType::EstimateType::MatrixType rotation = BossType::EstimateType::MatrixType::Zero();

    stream_ >> type;
    stream_ >> origin[0] >> origin[1] >> origin[2];
    stream_ >> rotation(0, 0) >> rotation(0, 1) >> rotation(0, 2) >> rotation(1, 0) >>
      rotation(1, 1) >> rotation(1, 2) >> rotation(2, 0) >> rotation(2, 1) >> rotation(2, 2);

    if (type < 0 || type > BossType::EstimateType::Type::Surfel) {
      throw std::runtime_error("ConverterActionVariableMatchable::readFromG2O|unknown type [" +
                               std::to_string(type) + "]");
    }

    BossType::EstimateType estimate((BossType::EstimateType::Type)(type), origin, rotation);

    _boss_object = new BossType();
    _boss_object->setGraphId(graph_id);
    _boss_object->setEstimate(estimate);
    _graph_ptr->addVariable(VariableBasePtr(_boss_object));
  }

  void ConverterActionVariableMatchable::writeFromBoss(std::ofstream& stream_, void* object_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionVariableMatchable::writeFromBoss|invalid graph pointer");

    _boss_object = reinterpret_cast<BossType*>(object_);
    assert(_boss_object != nullptr &&
           "ConverterActionVariableMatchable::writeFromBoss|invalid boss object");

    stream_ << _boss_object->graphId() << " ";

    // ia write estimate
    const auto& origin = _boss_object->estimate().origin();
    const auto& R      = _boss_object->estimate().rotation();

    stream_ << _boss_object->estimate().type() << " ";
    stream_ << origin[0] << " " << origin[1] << " " << origin[2] << " ";
    stream_ << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " " << R(1, 0) << " " << R(1, 1)
            << " " << R(1, 2) << " " << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << " ";

    if (_boss_object->status() == VariableBase::Status::Fixed) {
      stream_ << "\nFIX " << _boss_object->graphId();
    }
  }

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  void ConverterActionSE3PoseMatchableErrorFactor::readFromG2O(std::istream& stream_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionSE3PoseMatchableErrorFactor::readFromG2O|invalid graph pointer");

    // ia read the ID
    int graph_id_0 = -1;
    int graph_id_1 = -1;
    stream_ >> graph_id_0 >> graph_id_1;

    // ia read the measurement
    int type                                       = -1;
    BossType::MeasurementType::VectorType origin   = BossType::MeasurementType::VectorType::Zero();
    BossType::MeasurementType::MatrixType rotation = BossType::MeasurementType::MatrixType::Zero();

    stream_ >> type;
    stream_ >> origin[0] >> origin[1] >> origin[2];
    stream_ >> rotation(0, 0) >> rotation(0, 1) >> rotation(0, 2) >> rotation(1, 0) >>
      rotation(1, 1) >> rotation(1, 2) >> rotation(2, 0) >> rotation(2, 1) >> rotation(2, 2);

    if (type < 0 || type > BossType::MeasurementType::Type::Surfel) {
      throw std::runtime_error(
        "ConverterActionSE3PoseMatchableErrorFactor::readFromG2O|unknown type [" +
        std::to_string(type) + "]");
    }

    BossType::MeasurementType measurement(
      (BossType::MeasurementType::Type)(type), origin, rotation);

    // ia read the omega
    BossType::InformationMatrixType omega = BossType::InformationMatrixType::Identity();
    for (int i = 0; i < omega.rows() && stream_.good(); i++) {
      for (int j = i; j < omega.cols() && stream_.good(); j++) {
        stream_ >> omega(i, j);
        if (i != j){
          omega(j, i) = omega(i, j);
        }
      }
    }

    _boss_object = new BossType();
    _boss_object->setMeasurement(measurement);
    _boss_object->setInformationMatrix(omega);
    _boss_object->setVariableId(0, graph_id_0);
    _boss_object->setVariableId(1, graph_id_1);
    _graph_ptr->addFactor(FactorBasePtr(_boss_object));
  }

  void ConverterActionSE3PoseMatchableErrorFactor::writeFromBoss(std::ofstream& stream_,
                                                                 void* object_) {
    assert(_graph_ptr != nullptr &&
           "ConverterActionSE3PoseMatchableErrorFactor::writeFromBoss|invalid graph pointer");

    _boss_object = reinterpret_cast<BossType*>(object_);
    assert(_boss_object != nullptr &&
           "ConverterActionSE3PoseMatchableErrorFactor::writeFromBoss|invalid boss object");

    stream_ << _boss_object->variableId(0) << " " << _boss_object->variableId(1) << " ";
    // ia write estimate
    const auto& origin = _boss_object->measurement().origin();
    const auto& R      = _boss_object->measurement().rotation();

    stream_ << _boss_object->measurement().type() << " ";
    stream_ << origin[0] << " " << origin[1] << " " << origin[2] << " ";
    stream_ << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " " << R(1, 0) << " " << R(1, 1)
            << " " << R(1, 2) << " " << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << " ";

    const auto& omega = _boss_object->informationMatrix();
    for (int i = 0; i < omega.rows(); i++) {
      for (int j = i; j < omega.cols(); j++) {
        stream_ << omega(i, j) << " ";
      }
    }
  }

} // namespace srrg2_solver
