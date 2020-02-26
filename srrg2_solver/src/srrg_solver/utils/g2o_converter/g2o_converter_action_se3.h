#pragma once
#include "g2o_converter_action_base.h"
#include "srrg_solver/variables_and_factors/types_3d/instances.h"

namespace srrg2_solver {

  template <typename Scalar_>
  Isometry3_<Scalar_> readG2OIsometry3(std::istream& stream_) {
    Vector7_<Scalar_> reading = Vector7_<Scalar_>::Zero();
    for (size_t i = 0; i < 7; ++i) {
      stream_ >> reading[i];
    }

    Isometry3_<Scalar_> iso = Isometry3_<Scalar_>::Identity();
    iso                     = geometry3d::tqxyzq2t(reading);
    return iso;
  }

  template <typename Scalar_>
  void writeG2OIsometry3(std::ofstream& stream_, const Isometry3_<Scalar_>& iso_) {
    Vector7_<Scalar_> vector = Vector7_<Scalar_>::Zero();
    vector                   = geometry3d::t2tqxyzw(iso_);
    for (size_t i = 0; i < 7; ++i) {
      stream_ << vector[i] << " ";
    }
  }

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  class ConverterActionVariableSE3QuaternionRight
    : public ConverterAction_<VariableSE3QuaternionRight> {
  public:
    using Scalar    = VariableSE3QuaternionRight::EstimateType::Scalar;
    using BaseClass = ConverterAction_<VariableSE3QuaternionRight>;
    using BossType  = BaseClass::BossType;

    ConverterActionVariableSE3QuaternionRight(const FactorGraphPtr& graph_ptr_,
                                              G2OConverter* converter_ptr_) :
      BaseClass("VERTEX_SE3:QUAT", graph_ptr_, converter_ptr_) {
      // ia nothin to do
    }

    ~ConverterActionVariableSE3QuaternionRight() {
      // ia nothin to do
    }

    //! @brief this read from g2o file and creates a boss object
    void readFromG2O(std::istream& stream_) final;

    //! @brief this writes in a g2o file the boss object associated with this action
    void writeFromBoss(std::ofstream& stream_, void* object_) final;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  class ConverterActionVariableSE3QuaternionRightAD
    : public ConverterAction_<VariableSE3QuaternionRightAD> {
  public:
    using Scalar    = VariableSE3EulerRightAD::EstimateType::Scalar;
    using BaseClass = ConverterAction_<VariableSE3QuaternionRightAD>;
    using BossType  = BaseClass::BossType;

    ConverterActionVariableSE3QuaternionRightAD(const FactorGraphPtr& graph_ptr_,
                                                G2OConverter* converter_ptr_) :
      BaseClass("VERTEX_SE3:QUAT", graph_ptr_, converter_ptr_) {
      // ia nothin to do
    }

    ~ConverterActionVariableSE3QuaternionRightAD() {
      // ia nothin to do
    }

    //! @brief this read from g2o file and creates a boss object
    void readFromG2O(std::istream& stream_) final;

    void writeFromBoss(std::ofstream& stream_, void* object_) final;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  class ConverterActionVariableSE3EulerRight : public ConverterAction_<VariableSE3EulerRight> {
  public:
    using Scalar    = VariableSE3EulerRight::EstimateType::Scalar;
    using BaseClass = ConverterAction_<VariableSE3EulerRight>;
    using BossType  = BaseClass::BossType;

    ConverterActionVariableSE3EulerRight(const FactorGraphPtr& graph_ptr_,
                                         G2OConverter* converter_ptr_) :
      BaseClass("VERTEX_SE3:QUAT", graph_ptr_, converter_ptr_) {
      // ia nothin to do
    }

    ~ConverterActionVariableSE3EulerRight() {
      // ia nothin to do
    }

    //! @brief this read from g2o file and creates a boss object
    void readFromG2O(std::istream& stream_) final;

    void writeFromBoss(std::ofstream& stream_, void* object_) final;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  class ConverterActionVariableSE3EulerRightAD : public ConverterAction_<VariableSE3EulerRightAD> {
  public:
    using Scalar    = VariableSE3EulerRightAD::EstimateType::Scalar;
    using BaseClass = ConverterAction_<VariableSE3EulerRightAD>;
    using BossType  = BaseClass::BossType;

    ConverterActionVariableSE3EulerRightAD(const FactorGraphPtr& graph_ptr_,
                                           G2OConverter* converter_ptr_) :
      BaseClass("VERTEX_SE3:QUAT", graph_ptr_, converter_ptr_) {
      // ia nothin to do
    }

    ~ConverterActionVariableSE3EulerRightAD() {
      // ia nothin to do
    }

    //! @brief this read from g2o file and creates a boss object
    void readFromG2O(std::istream& stream_) final;

    void writeFromBoss(std::ofstream& stream_, void* object_) final;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  class ConverterActionVariableSE3EulerLeft : public ConverterAction_<VariableSE3EulerLeft> {
  public:
    using Scalar    = VariableSE3EulerLeft::EstimateType::Scalar;
    using BaseClass = ConverterAction_<VariableSE3EulerLeft>;
    using BossType  = BaseClass::BossType;

    ConverterActionVariableSE3EulerLeft(const FactorGraphPtr& graph_ptr_,
                                        G2OConverter* converter_ptr_) :
      BaseClass("VERTEX_SE3:EULERPERT", graph_ptr_, converter_ptr_) {
      // ia nothin to do
    }

    ~ConverterActionVariableSE3EulerLeft() {
      // ia nothin to do
    }

    //! @brief this read from g2o file and creates a boss object
    void readFromG2O(std::istream& stream_) final;

    void writeFromBoss(std::ofstream& stream_, void* object_) final;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  class ConverterActionVariableSE3EulerLeftAD : public ConverterAction_<VariableSE3EulerLeftAD> {
  public:
    using Scalar    = VariableSE3EulerLeftAD::EstimateType::Scalar;
    using BaseClass = ConverterAction_<VariableSE3EulerLeftAD>;
    using BossType  = BaseClass::BossType;

    ConverterActionVariableSE3EulerLeftAD(const FactorGraphPtr& graph_ptr_,
                                          G2OConverter* converter_ptr_) :
      BaseClass("VERTEX_SE3:EULERPERT", graph_ptr_, converter_ptr_) {
      // ia nothin to do
    }

    ~ConverterActionVariableSE3EulerLeftAD() {
      // ia nothin to do
    }

    //! @brief this read from g2o file and creates a boss object
    void readFromG2O(std::istream& stream_) final;

    void writeFromBoss(std::ofstream& stream_, void* object_) final;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  class ConverterActionSE3PoseOffset : public ConverterActionBase {
  public:
    using BaseClass = ConverterActionBase;
    using BossType  = VariableSE3QuaternionRight; // ia hardcoded type since it's an action :)
    ConverterActionSE3PoseOffset(const FactorGraphPtr& graph_ptr_, G2OConverter* converter_ptr_) :
      BaseClass("PARAMS_SE3OFFSET", graph_ptr_, converter_ptr_) {
      _hash = 1;
    }

    ~ConverterActionSE3PoseOffset() {
    }

    // ia this is the cpp
    void readFromG2O(std::istream& stream_) final;

    void writeFromBoss(std::ofstream& stream_, void* object_) final {
      // ia does nothin
    }

  protected:
    BossType* _boss_object = nullptr;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  class ConverterActionSE3PoseOffsetAD : public ConverterActionBase {
  public:
    using BaseClass = ConverterActionBase;
    using BossType  = VariableSE3QuaternionRightAD; // ia hardcoded type since it's an action :)
    ConverterActionSE3PoseOffsetAD(const FactorGraphPtr& graph_ptr_, G2OConverter* converter_ptr_) :
      BaseClass("PARAMS_SE3OFFSET", graph_ptr_, converter_ptr_) {
      _hash = 1;
    }

    ~ConverterActionSE3PoseOffsetAD() {
    }

    // ia this is the cpp
    void readFromG2O(std::istream& stream_) final;

    void writeFromBoss(std::ofstream& stream_, void* object_) final {
      // ia does nothin
    }

  protected:
    BossType* _boss_object = nullptr;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  class ConverterActionSE3PosePoseGeodesicErrorFactor
    : public ConverterAction_<SE3PosePoseGeodesicErrorFactor> {
  public:
    using Scalar    = SE3PosePoseGeodesicErrorFactor::MeasurementType::Scalar;
    using BaseClass = ConverterAction_<SE3PosePoseGeodesicErrorFactor>;
    using BossType  = BaseClass::BossType;

    ConverterActionSE3PosePoseGeodesicErrorFactor(const FactorGraphPtr& graph_ptr_,
                                                            G2OConverter* converter_ptr_) :
      BaseClass("EDGE_SE3:QUAT", graph_ptr_, converter_ptr_) {
      // ia nothin to do
    }

    ~ConverterActionSE3PosePoseGeodesicErrorFactor() {
      // ia nothin to do
    }

    //! @brief this read from g2o file and creates a boss object
    void readFromG2O(std::istream& stream_) final;

    void writeFromBoss(std::ofstream& stream_, void* object_) final;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  class ConverterActionSE3PosePoseChordalEulerLeftErrorFactor
    : public ConverterAction_<SE3PosePoseChordalEulerLeftErrorFactor> {
  public:
    using Scalar    = SE3PosePoseChordalEulerLeftErrorFactor::MeasurementType::Scalar;
    using BaseClass = ConverterAction_<SE3PosePoseChordalEulerLeftErrorFactor>;
    using BossType  = BaseClass::BossType;

    ConverterActionSE3PosePoseChordalEulerLeftErrorFactor(const FactorGraphPtr& graph_ptr_,
                                                      G2OConverter* converter_ptr_) :
      BaseClass("EDGE_SE3:CHORD-A", graph_ptr_, converter_ptr_) {
      // ia nothin to do
    }

    ~ConverterActionSE3PosePoseChordalEulerLeftErrorFactor() {
      // ia nothin to do
    }

    //! @brief this read from g2o file and creates a boss object
    void readFromG2O(std::istream& stream_) final;

    void writeFromBoss(std::ofstream& stream_, void* object_) final;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  class ConverterActionSE3PosePointOffsetErrorFactor
    : public ConverterAction_<SE3PosePointOffsetErrorFactor> {
  public:
    using Scalar    = SE3PosePointOffsetErrorFactor::MeasurementType::Scalar;
    using BaseClass = ConverterAction_<SE3PosePointOffsetErrorFactor>;
    using BossType  = BaseClass::BossType;

    ConverterActionSE3PosePointOffsetErrorFactor(const FactorGraphPtr& graph_ptr_,
                                                 G2OConverter* converter_ptr_) :
      BaseClass("EDGE_SE3_TRACKXYZ", graph_ptr_, converter_ptr_) {
      // ia nothin to do
    }

    ~ConverterActionSE3PosePointOffsetErrorFactor() {
      // ia nothin to do
    }

    //! @brief this read from g2o file and creates a boss object - defined in the cpp
    void readFromG2O(std::istream& stream_) final;

    void writeFromBoss(std::ofstream& stream_, void* object_) final;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  class ConverterActionVariablePoint3AD : public ConverterAction_<VariablePoint3AD> {
  public:
    using Scalar    = VariablePoint3AD::EstimateType::Scalar;
    using BaseClass = ConverterAction_<VariablePoint3AD>;
    using BossType  = BaseClass::BossType;

    ConverterActionVariablePoint3AD(const FactorGraphPtr& graph_ptr_,
                                    G2OConverter* converter_ptr_) :
      BaseClass("VERTEX_TRACKXYZ", graph_ptr_, converter_ptr_) {
      // ia nothin to do
    }

    ~ConverterActionVariablePoint3AD() {
      // ia nothin to do
    }

    //! @brief this read from g2o file and creates a boss object
    void readFromG2O(std::istream& stream_) final;

    void writeFromBoss(std::ofstream& stream_, void* object_) final;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  class ConverterActionVariablePoint3 : public ConverterAction_<VariablePoint3> {
  public:
    using Scalar    = VariablePoint3::EstimateType::Scalar;
    using BaseClass = ConverterAction_<VariablePoint3>;
    using BossType  = BaseClass::BossType;

    ConverterActionVariablePoint3(const FactorGraphPtr& graph_ptr_, G2OConverter* converter_ptr_) :
      BaseClass("VERTEX_TRACKXYZ", graph_ptr_, converter_ptr_) {
      // ia nothin to do
    }

    ~ConverterActionVariablePoint3() {
      // ia nothin to do
    }

    //! @brief this read from g2o file and creates a boss object
    void readFromG2O(std::istream& stream_) final;

    void writeFromBoss(std::ofstream& stream_, void* object_) final;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  class ConverterActionVariableMatchable : public ConverterAction_<VariableMatchable> {
  public:
    using Scalar    = VariableMatchable::EstimateType::Scalar;
    using BaseClass = ConverterAction_<VariableMatchable>;
    using BossType  = BaseClass::BossType;

    ConverterActionVariableMatchable(const FactorGraphPtr& graph_ptr_,
                                     G2OConverter* converter_ptr_) :
      BaseClass("VERTEX_MATCHABLE", graph_ptr_, converter_ptr_) {
      // ia nothin to do
    }

    ~ConverterActionVariableMatchable() {
      // ia nothin to do
    }

    //! @brief this read from g2o file and creates a boss object
    void readFromG2O(std::istream& stream_) final;

    void writeFromBoss(std::ofstream& stream_, void* object_) final;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  class ConverterActionSE3PoseMatchableErrorFactor
    : public ConverterAction_<SE3PoseMatchableEulerLeftErrorFactor> {
  public:
    using Scalar    = SE3PoseMatchableEulerLeftErrorFactor::MeasurementType::Scalar;
    using BaseClass = ConverterAction_<SE3PoseMatchableEulerLeftErrorFactor>;
    using BossType  = BaseClass::BossType;

    ConverterActionSE3PoseMatchableErrorFactor(const FactorGraphPtr& graph_ptr_,
                                               G2OConverter* converter_ptr_) :
      BaseClass("EDGE_SE3_MATCHABLE", graph_ptr_, converter_ptr_) {
      // ia nothin to do
    }

    ~ConverterActionSE3PoseMatchableErrorFactor() {
      // ia nothin to do
    }

    //! @brief this read from g2o file and creates a boss object
    void readFromG2O(std::istream& stream_) final;

    void writeFromBoss(std::ofstream& stream_, void* object_) final;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace srrg2_solver
