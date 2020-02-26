#pragma once
#include "srrg_solver/solver_core/factor_graph.h"

namespace srrg2_solver {

  //! @brief forward declaration of g2o converter (just to have a raw ptr of it)
  class G2OConverter;

  class ConverterActionBase {
  public:
    //! @brief object life
    ConverterActionBase() = delete;
    ConverterActionBase(const std::string& tag_,
                        const FactorGraphPtr& graph_ptr_,
                        G2OConverter* converter_ptr_) :
      _tag(tag_),
      _graph_ptr(graph_ptr_),
      _converter_ptr(converter_ptr_) {
      _hash = 0;
      assert((_graph_ptr != nullptr) &&
             "ConverterActionBase::ConverterActionBase|invalid converter "
             " pointer, what's going on???");
    }

    virtual ~ConverterActionBase() {
    }

    //! @brief accessors
    inline const std::string& tag() const {
      return _tag;
    }
    inline const uint64_t& hash() const {
      return _hash;
    }

    //! @brief this read from g2o file and creates a boss object
    virtual void readFromG2O(std::istream& stream_) = 0;

    virtual void writeFromBoss(std::ofstream& stream_, void* object_) = 0;

  protected:
    const std::string _tag; // ia tag is for G2O objects
    uint64_t _hash;         // ia hash is for BOSS objects

    // ia pointer to the factor graph to populate
    FactorGraphPtr _graph_ptr = nullptr;

    // ia pointer to the converter that creates the action
    G2OConverter* _converter_ptr = nullptr;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  class ConverterActionTagFixed : public ConverterActionBase {
  public:
    using BaseClass = ConverterActionBase;
    ConverterActionTagFixed(const FactorGraphPtr& graph_ptr_, G2OConverter* converter_ptr_) :
      BaseClass("FIX", graph_ptr_, converter_ptr_) {
    }

    ~ConverterActionTagFixed() {
    }

    void readFromG2O(std::istream& stream_) final {
      assert(_graph_ptr != nullptr && "ConverterActionTagFixed::readFromG2O|invalid graph ptr");
      int variable_id = -1;
      stream_ >> variable_id;

      VariableBase* variable = _graph_ptr->variable(variable_id);
      if (!variable) {
        throw std::runtime_error("ConverterActionTagFixed::readFromG2O|invalid variable id");
      }

      std::cerr << "ConverterActionTagFixed::readFromG2O|fixed variable #" << variable_id
                << std::endl;
      variable->setStatus(VariableBase::Status::Fixed);
    }

    void writeFromBoss(std::ofstream& stream_, void* object_) final {
      // ia does nothin
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  template <typename BossObjectType_>
  class ConverterAction_ : public ConverterActionBase {
  public:
    using BossType = BossObjectType_;

    ConverterAction_(const std::string& tag_,
                     const FactorGraphPtr& graph_ptr_,
                     G2OConverter* converter_ptr_) :
      ConverterActionBase(tag_, graph_ptr_, converter_ptr_) {
      _hash = typeid(BossType).hash_code();
      //      _hash = typeid(*this).hash_code();
    }

    virtual ~ConverterAction_() {
      // ia empty
    }

    //! @brief this read from g2o file and creates a boss object
    void readFromG2O(std::istream& stream_) override {
    }

    void writeFromBoss(std::ofstream& stream_, void* object_) override {
    }

    inline BossType* bossObject() {
      return _boss_object;
    }

  protected:
    BossType* _boss_object = nullptr;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace srrg2_solver
