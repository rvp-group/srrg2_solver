#pragma once
#include <list>
#include <srrg_boss/object_data.h>
#include <srrg_data_structures/abstract_map_view.h>
#include <srrg_geometry/ad.h>
#include <srrg_viewer/drawable_base.h>

namespace srrg2_solver {

  using namespace srrg2_core;

  //! the motha of all variables
  //! a variable has
  //! - an id (-1 when invalid)
  //! - an estimate (to be assigned in the specialized class
  //! Variable_<EstimateType>
  //! - a "fixed" flag, that if true inhibits the variable from being part in
  //! the optimization
  //! - a stack of values, implemented through push() and pop(). This is useful
  //! to perform
  //!   some temporary operations on a portion of the graph
  //!
  //! a variable implements
  //! - a perturbationDim() method, that returns the dimension of the
  //! perturbation (chart)
  //! - a method to apply the perturbation from an array of float
  //!   (having at minimum dimension perturbationDim

  class VariableBase : public Identifiable, public DrawableBase {
  public:
    enum Status { Active = 0, NonActive = 1, Fixed = 2 };

    using Id = int64_t;

  protected:
    friend class Solver;
    template <typename V, typename... VRest>
    friend struct VariablePtrTuple_;

  public:
    // ! resets the value of the variable to zero/identity....
    virtual void setZero() = 0;

    // ! returns the dimension of the pertubation
    virtual int perturbationDim() const = 0;

    // ! applies the perturbation to the estimate
    virtual void applyPerturbationRaw(const float* pert_data) = 0;

    // ! returns the id
    inline Id graphId() const {
      return _graph_id;
    }

    //! sets the id
    inline void setGraphId(Id graph_id_) {
      _graph_id = graph_id_;
    }

    //! queries the status
    inline Status status() const {
      return _status;
    }

    //! sets the  status
    inline void setStatus(Status status_) {
      _status = status_;
    }

    //! pushes the current estimate to the stack
    virtual void push() = 0;

    //! pops the current estimate from the stack
    virtual void pop() = 0;

    //! keeps the current estimate, but discards the top of the stack
    //! throws away the most recently saved value
    virtual void discardTop() = 0;

    //! draws itself on a potential canvas - no need to target anything but the
    //! core. overridden by different types of variables, to allow drawing
    //! without any stupid cast
    void draw(ViewerCanvasPtr canvas_) const override {
      if (!canvas_)
        throw std::runtime_error("Variable::draw|invalid canvas");
      std::cerr << "VariableBase::draw not implemented" << std::endl;
    }

    inline bool tainted() const {return _tainted;}
    inline void untaint() {_tainted = false;}
  protected:
    inline void taint() {_tainted = true;}
    inline int getId() {
      return Identifiable::getId();
    }
    inline void setId(int id_) {
      Identifiable::setId(id_);
    }
    Id _graph_id       = -1;
    int _hessian_index = -1;
    Status _status     = Active;
    bool _tainted      = true;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using VariableBasePtr = std::shared_ptr<VariableBase>;

  //! @brief intermediate variable required to interact with generic
  //!        non-Eigen objects. Serialization and Deserialization must be specified
  template <int PerturbationDim_, template <typename Scalar_> typename EstimateType_>
  class VariableGeneric_ : public VariableBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseVariableType               = VariableGeneric_<PerturbationDim_, EstimateType_>;
    using EstimateType                   = EstimateType_<float>;      // ia this sucks
    using ADEstimateType                 = EstimateType_<DualValuef>; // ia this sucks
    static constexpr int PerturbationDim = PerturbationDim_;
    using PerturbationVectorType =
      typename Eigen::Matrix<float, PerturbationDim_, 1>; // ia this sucks

    //! @brief object life (all static stuff)
    VariableGeneric_() {
      // ia nothin to do here
    }
    virtual ~VariableGeneric_(){
      // ia nothin to do here
    };

    //! @brief perturbation dimension
    int perturbationDim() const override {
      return PerturbationDim;
    }

    //! @brief method of the base class, that calls the typed
    //!        applyPerturbation of the derived class
    void applyPerturbationRaw(const float* pert_data_) override {
      Eigen::Map<const PerturbationVectorType> pert(pert_data_);
      applyPerturbation(pert);
    }

    //! @brief sets the estimate to zero, implemented in the specialized case
    virtual void setZero() = 0;

    //! @brief sets estimate
    virtual void setEstimate(const EstimateType& est_) {
      _estimate = est_;
      _tainted=true;
    }

    //! @brief accessor for the estimate
    inline const EstimateType& estimate() const {
      return _estimate;
    }

    //! @brief define this in the derived classes
    virtual void applyPerturbation(const PerturbationVectorType& perturbation_) = 0;

    //! @brief pushes the variable on the variable stack
    //!        (like openGL does with attributes and matrices)
    void push() override;

    //! @brief pops the variable from the variable stack
    //!        (like openGL does with attributes and matrices)
    void pop() override;

    //! @brief keeps the current estimate, but discards the top of the stack
    //!        (throws away the most recently saved value)
    void discardTop() override;

    //! @brief serialization of the variable through BOSS.
    //!        This must be specialized in the derived class
    void serialize(ObjectData& odata, IdContext& context) override {
      throw std::runtime_error("VariableGeneric_::serialize|you must implement this function");
    }

    //! @brief deserialization of the variable through BOSS.
    //!        This must be specialized in the derived class
    void deserialize(ObjectData& odata, IdContext& context) override {
      throw std::runtime_error("VariableGeneric_::deserialize|you must implement this function");
    }

  protected:
    EstimateType _estimate = EstimateType::Identity();
    std::list<EstimateType, Eigen::aligned_allocator<EstimateType>> _stack;
  };

  //! @brief specialization of a variable built on an EstimateType_
  //!        the "numeric" value of EstimateType_ should be a template -
  //!        e.g. Isometry3_<t> - as it is overridden by autodiff if necessary
  //!        the estimate is materialized inside as EstimateType_<float>
  //!        PerturbationDim_: the dimension of the perturbation
  //!
  //!        This class implements most of the variable interface, including the stack
  //!        the only method you need to specialize in a derived class are
  //!         - applyPerturvation(const PerturbationVectorType& pert_)
  //!           that takes an eigen vector of float and applies the perturbation to the
  //!           current estimate
  //!         - setZero() that resets the esitmate
  //!        Serialization and deserialization is also implemented
  //!        ASSUMING THAT ESTIMATE IS AN EIGEN OBJECT. If this is not your favourite whiskey than
  //!        you should derive from VariableGeneric_.
  template <int PerturbationDim_, template <typename Scalar_> typename EstimateType_>
  class Variable_ : public VariableGeneric_<PerturbationDim_, EstimateType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BaseVariableType           = Variable_<PerturbationDim_, EstimateType_>;
    using EstimateType               = EstimateType_<float>;
    using ADEstimateType             = EstimateType_<DualValuef>;
    static const int PerturbationDim = PerturbationDim_;
    using PerturbationVectorType     = typename Eigen::Matrix<float, PerturbationDim_, 1>;

    //! @brief object life, everything is static
    Variable_() {
      // ia nothin to do
    }

    virtual ~Variable_() {
      // ia nothin to do
    }

    void setZero() override {
      // ia this must be specified in the derived class
    }

    // define this in the derived classes
    void applyPerturbation(const PerturbationVectorType& perturbation) override {
      // ia this must be specified in the derived class
    }

    //! @brief serialization assuming eigen estimate - implemented in the _impl.cpp
    //!        this CANNOT be overridden to avoid strange behaviours
    void serialize(ObjectData& odata, IdContext& context) final;

    //! @brief deserialization assuming eigen estimate - implemented in the _impl.cpp
    //!        this CANNOT be overridden to avoid strange behaviours
    void deserialize(ObjectData& odata, IdContext& context) final;
  };

  using IdVariablePtrContainer = AbstractMapView_<VariableBase::Id, VariableBase const*>;
  using IdVariablePair         = std::pair<VariableBase::Id, VariableBase const*>;

} // namespace srrg2_solver

//#include "variable.hpp"
