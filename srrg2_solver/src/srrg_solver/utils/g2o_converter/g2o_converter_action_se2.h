#pragma once
#include "g2o_converter_action_base.h"
#include "srrg_solver/variables_and_factors/types_2d/instances.h"

namespace srrg2_solver {

  // ia aux function to read and write isometries
  template <typename Scalar_>
  Isometry2_<Scalar_> readG2OIsometry2(std::istream& stream_) {
    Vector3_<Scalar_> reading = Vector3_<Scalar_>::Zero();
    for (size_t i = 0; i < 3; ++i) {
      stream_ >> reading[i];
    }

    Isometry2_<Scalar_> iso = Isometry2_<Scalar_>::Identity();
    iso                     = geometry2d::v2t(reading);
    return iso;
  }

  template <typename Scalar_>
  void writeG2OIsometry2(std::ofstream& stream_, const Isometry2_<Scalar_>& iso_) {
    Vector3_<Scalar_> vector = Vector3_<Scalar_>::Zero();
    vector                   = geometry2d::t2v(iso_);
    for (size_t i = 0; i < 3; ++i) {
      stream_ << vector[i] << " ";
    }
  }

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  class ConverterActionVariableSE2RightAD : public ConverterAction_<VariableSE2RightAD> {
  public:
    using Scalar    = VariableSE2RightAD::EstimateType::Scalar;
    using BaseClass = ConverterAction_<VariableSE2RightAD>;
    using BossType  = BaseClass::BossType;

    ConverterActionVariableSE2RightAD(const FactorGraphPtr& graph_ptr_,
                                      G2OConverter* converter_ptr_) :
      BaseClass("VERTEX_SE2", graph_ptr_, converter_ptr_) {
      // ia nothin to do
    }

    virtual ~ConverterActionVariableSE2RightAD() {
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
  class ConverterActionVariableSE2Right : public ConverterAction_<VariableSE2Right> {
  public:
    using Scalar    = VariableSE2RightAD::EstimateType::Scalar;
    using BaseClass = ConverterAction_<VariableSE2Right>;
    using BossType  = BaseClass::BossType;

    ConverterActionVariableSE2Right(const FactorGraphPtr& graph_ptr_,
                                    G2OConverter* converter_ptr_) :
      BaseClass("VERTEX_SE2", graph_ptr_, converter_ptr_) {
      // ia nothin to do
    }

    virtual ~ConverterActionVariableSE2Right() {
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
  class ConverterActionVariableSE2LeftAD : public ConverterAction_<VariableSE2LeftAD> {
  public:
    using Scalar    = VariableSE2LeftAD::EstimateType::Scalar;
    using BaseClass = ConverterAction_<VariableSE2LeftAD>;
    using BossType  = BaseClass::BossType;

    ConverterActionVariableSE2LeftAD(const FactorGraphPtr& graph_ptr_,
                                     G2OConverter* converter_ptr_) :
      BaseClass("VERTEX_SE2", graph_ptr_, converter_ptr_) {
      // ia nothin to do
    }

    virtual ~ConverterActionVariableSE2LeftAD() {
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
  class ConverterActionVariableSE2Left : public ConverterAction_<VariableSE2Left> {
  public:
    using Scalar    = VariableSE2LeftAD::EstimateType::Scalar;
    using BaseClass = ConverterAction_<VariableSE2Left>;
    using BossType  = BaseClass::BossType;

    ConverterActionVariableSE2Left(const FactorGraphPtr& graph_ptr_, G2OConverter* converter_ptr_) :
      BaseClass("VERTEX_SE2", graph_ptr_, converter_ptr_) {
      // ia nothin to do
    }

    virtual ~ConverterActionVariableSE2Left() {
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
  class ConverterActionVariablePoint2AD : public ConverterAction_<VariablePoint2AD> {
  public:
    using Scalar    = VariablePoint2AD::EstimateType::Scalar;
    using BaseClass = ConverterAction_<VariablePoint2AD>;
    using BossType  = BaseClass::BossType;

    ConverterActionVariablePoint2AD(const FactorGraphPtr& graph_ptr_,
                                    G2OConverter* converter_ptr_) :
      BaseClass("VERTEX_XY", graph_ptr_, converter_ptr_) {
      // ia nothin to do
    }

    virtual ~ConverterActionVariablePoint2AD() {
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
  class ConverterActionVariablePoint2 : public ConverterAction_<VariablePoint2> {
  public:
    using Scalar    = VariablePoint2::EstimateType::Scalar;
    using BaseClass = ConverterAction_<VariablePoint2>;
    using BossType  = BaseClass::BossType;

    ConverterActionVariablePoint2(const FactorGraphPtr& graph_ptr_, G2OConverter* converter_ptr_) :
      BaseClass("VERTEX_XY", graph_ptr_, converter_ptr_) {
      // ia nothin to do
    }

    virtual ~ConverterActionVariablePoint2() {
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
  class ConverterActionSE2PosePoseGeodesicErrorFactor
    : public ConverterAction_<SE2PosePoseGeodesicErrorFactor> {
  public:
    using Scalar    = SE2PosePoseGeodesicErrorFactor::MeasurementType::Scalar;
    using BaseClass = ConverterAction_<SE2PosePoseGeodesicErrorFactor>;
    using BossType  = BaseClass::BossType;

    ConverterActionSE2PosePoseGeodesicErrorFactor(const FactorGraphPtr& graph_ptr_,
                                                  G2OConverter* converter_ptr_) :
      BaseClass("EDGE_SE2", graph_ptr_, converter_ptr_) {
      // ia nothin to do
    }

    ~ConverterActionSE2PosePoseGeodesicErrorFactor() {
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
  class ConverterActionSE2PosePointErrorFactor : public ConverterAction_<SE2PosePointErrorFactor> {
  public:
    using Scalar    = SE2PosePointErrorFactor::MeasurementType::Scalar;
    using BaseClass = ConverterAction_<SE2PosePointErrorFactor>;
    using BossType  = BaseClass::BossType;

    ConverterActionSE2PosePointErrorFactor(const FactorGraphPtr& graph_ptr_,
                                           G2OConverter* converter_ptr_) :
      BaseClass("EDGE_SE2_XY", graph_ptr_, converter_ptr_) {
      // ia nothin to do
    }

    ~ConverterActionSE2PosePointErrorFactor() {
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
  class ConverterActionSE2PosePointBearingErrorFactor
    : public ConverterAction_<SE2PosePointBearingErrorFactor> {
  public:
    using Scalar    = SE2PosePointBearingErrorFactor::MeasurementType::Scalar;
    using BaseClass = ConverterAction_<SE2PosePointBearingErrorFactor>;
    using BossType  = BaseClass::BossType;

    ConverterActionSE2PosePointBearingErrorFactor(const FactorGraphPtr& graph_ptr_,
                                                  G2OConverter* converter_ptr_) :
      BaseClass("EDGE_BEARING_SE2_XY", graph_ptr_, converter_ptr_) {
      // ia nothin to do
    }

    ~ConverterActionSE2PosePointBearingErrorFactor() {
      // ia nothin to do
    }

    //! @brief this read from g2o file and creates a boss object
    void readFromG2O(std::istream& stream_) final;

    //! @brief this writes in a g2o file the boss object associated with this action
    void writeFromBoss(std::ofstream& stream_, void* object_) final;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace srrg2_solver
