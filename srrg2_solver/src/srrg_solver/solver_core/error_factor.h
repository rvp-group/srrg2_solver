#pragma once
#include "measurement_owner.h"
#include "factor.h"
#include "variable_ptr_tuple.h"

namespace srrg2_solver {

  //! @brief middle class for a factor for a  variable problem constructed out of error
  //!        functions (ad and non ad). Derive from this if your measurement is a generic non-Eigen
  //!        Object. you must implement serialization and deserialization
  template <int ErrorDim_, typename... VariableTypes_>
  class ErrorFactor_ : public Factor_<VariablePtrTuple_<VariableTypes_...>> {
  public:
    using ThisType          = ErrorFactor_<ErrorDim_, VariableTypes_...>;
    using VariableTupleType = VariablePtrTuple_<VariableTypes_...>;
    using BaseType          = Factor_<VariableTupleType>;
    // ! number of variables involved in the factor
    static constexpr int NumVariables = BaseType::NumVariables;
    // ! error dimensionality
    static constexpr int ErrorDim = ErrorDim_;
    // ! type of the information matrix
    using InformationMatrixType = Eigen::Matrix<float, ErrorDim, ErrorDim>;
    // ! type of the error vector
    using ErrorVectorType = Eigen::Matrix<float, ErrorDim, 1>;
    // ! total perturbation dimension
    static constexpr int TotalPerturbationDim = BaseType::TotalPerturbationDim;
    //! type of the (total) perturbation vector
    using TotalPerturbationVectorType = typename BaseType::TotalPerturbationVectorType;
    //! type of the (total) jacobian
    //  obtained by stacking horizontally the individual jacobians of the variables
    using TotalJacobianMatrixType = typename Eigen::Matrix<float, ErrorDim, TotalPerturbationDim>;

    template <typename ThisType_, int r, int c>
    friend struct ColUpdater;

    template <typename ThisType_, int r, int c>
    friend struct RowUpdater;

    template <typename ThisType_, int r, int c>
    friend void ErrorFactor_internalColUpdate(ThisType_& self);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //! @brief object life - set everything to zero
    ErrorFactor_() : BaseType(){
      _information_matrix.setIdentity();
    }

    virtual ~ErrorFactor_(){
      
    }
    //! dimension of the ith variable
    template <int i>
    static constexpr int perturbationDim() {
      return VariableTupleType::template perturbationDim<i>();
    }

    //! offset of the ith variable perturbation in the jacobian
    template <int i>
    static constexpr int perturbationOffset() {
      return VariableTupleType::template perturbationOffset<i>();
    }

    //! type of the ith jacobian
    template <int idx>
    using JacobianMatrixType = typename Eigen::Map<
      const Eigen::Matrix<float, ErrorDim, ThisType::template perturbationDim<idx>()>>;

    //! read access to the ith jacobian
    template <int idx>
    inline const JacobianMatrixType<idx> jacobian() const {
      return JacobianMatrixType<idx>(_J.data() + ThisType::template perturbationOffset<idx>() * ErrorDim);
      // return _J.template block<ErrorDim, perturbationDim<idx>() >(0, perturbationOffset<idx>());
    }

    template <int idx>
    using MutableJacobianMatrixType = typename Eigen::Map<
      Eigen::Matrix<float, ErrorDim, ThisType::template perturbationDim<idx>()>>;

    template <int idx>
    inline MutableJacobianMatrixType<idx> jacobian() {
      return MutableJacobianMatrixType<idx>(_J.data() + ThisType::template perturbationOffset<idx>() * ErrorDim);
    }

    // define this in the derived classes to write the jacobian where it should be
    // void compute(bool error_only=false);

    // returns the error
    inline const ErrorVectorType& error() const {
      return _e;
    }

    // accessor to the total jacobian
    inline const TotalJacobianMatrixType& totalJacobian() const {
      return _J;
    }

    //! @brief checks if the computation is good for this factor
    bool isValid() const override;

    //! @brief omega accessor
    inline const InformationMatrixType& informationMatrix() const {
      return _information_matrix;
    }

    //! @brief omega setter
    inline void setInformationMatrix(const InformationMatrixType& information_matrix) {
      _information_matrix = information_matrix;
    }

    // ! @brief compute error vector and Jacobian
    // this function have to be overrided in the derived error factor
    virtual void errorAndJacobian(bool error_only) = 0;

    //! @brief overridden from FactorBase
    void compute(bool chi_only = false, bool force = false) final;

    void serialize(ObjectData& odata, IdContext& context) override;

    void deserialize(ObjectData& odata, IdContext& context) override;

    //! @brief aux functions
    template <int r, int c>
    inline void _updateHBlock();

    //! @brief aux functions
    template <int r>
    inline void _updateBBlock();

    //! @brief aux functions
    inline void updateH();

  protected:
    //! @brief attributes
    ErrorVectorType _e;
    TotalJacobianMatrixType _J;
    InformationMatrixType _information_matrix;

    bool _is_valid = true;
  };

} // namespace srrg2_solver
