#pragma once
#include "factor.h"
#include "measurement_owner.h"
#include "variable_ptr_tuple.h"

namespace srrg2_solver {

  /*! @brief Error factor class, which use variatic templates to handle an arbitrary number of
     variables that might be connected to the factor - e.g. ErrorFactor_<3, VariableSE2,
     VariablePoint2>. To define your error factor you must specify in the template the error
     dimension and the variable types. In the derived class you must implement:

      - errorAndJacobian() which evaluate the error vector and compute the jacobian matrix
  */
  template <int ErrorDim_, typename... VariableTypes_>
  class ErrorFactor_ : public Factor_<VariablePtrTuple_<VariableTypes_...>> {
  public:
    using ThisType          = ErrorFactor_<ErrorDim_, VariableTypes_...>;
    using VariableTupleType = VariablePtrTuple_<VariableTypes_...>; /*!< Container
                                                                      of variable
                                                                      instanciated
                                                                      using variatic templates
                                                                    */
    using BaseType = Factor_<VariableTupleType>;

    static constexpr int NumVariables =
      BaseType::NumVariables; /*!< Number of variables involved in the factor */

    static constexpr int ErrorDim = ErrorDim_; /*!< Error dimensionality */

    using InformationMatrixType = Eigen::Matrix<float, ErrorDim, ErrorDim>; /*!< Type of the
                                                                              information matrix */

    using ErrorVectorType = Eigen::Matrix<float, ErrorDim, 1>; /*!< Type of the error vector */

    static constexpr int TotalPerturbationDim = BaseType::TotalPerturbationDim; /*!< Total
                                                                                  perturbation
                                                                                  dimension */
    using TotalPerturbationVectorType =
      typename BaseType::TotalPerturbationVectorType; /*!<
                                                         Type
                                                         of
                                                         the total
                                                         perturbation
                                                         vector */

    using TotalJacobianMatrixType =
      typename Eigen::Matrix<float, ErrorDim, TotalPerturbationDim>; /*!< Type
                                                                       of the
                                                                       total jacobian
                                                                       obtained by stacking
                                                                       horizontally the individual
                                                                       jacobians of the variables */

    template <typename ThisType_, int r, int c>
    friend struct ColUpdater;

    template <typename ThisType_, int r, int c>
    friend struct RowUpdater;

    template <typename ThisType_, int r, int c>
    friend void ErrorFactor_internalColUpdate(ThisType_& self);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //! @brief object life - set everything to zero
    ErrorFactor_() : BaseType() {
      _information_matrix.setIdentity();
    }

    virtual ~ErrorFactor_() {
    }
    /*! @return Dimension of the i-th variable*/
    template <int i>
    static constexpr int perturbationDim() {
      return VariableTupleType::template perturbationDim<i>();
    }

    /*! @return Offset of the i-th variable perturbation in the jacobian */
    template <int i>
    static constexpr int perturbationOffset() {
      return VariableTupleType::template perturbationOffset<i>();
    }

    /*! @return Type of the i-th jacobian */
    template <int i>
    using JacobianMatrixType = typename Eigen::Map<
      const Eigen::Matrix<float, ErrorDim, ThisType::template perturbationDim<i>()>>;

    /*! @return The i-th jacobian (read only) */
    template <int i>
    inline const JacobianMatrixType<i> jacobian() const {
      return JacobianMatrixType<i>(_J.data() +
                                   ThisType::template perturbationOffset<i>() * ErrorDim);
    }
    /*! Type of the i-th jacobian */
    template <int i>
    using MutableJacobianMatrixType =
      typename Eigen::Map<Eigen::Matrix<float, ErrorDim, ThisType::template perturbationDim<i>()>>;

    /*! @return The i-th jacobian */
    template <int i>
    inline MutableJacobianMatrixType<i> jacobian() {
      return MutableJacobianMatrixType<i>(_J.data() +
                                          ThisType::template perturbationOffset<i>() * ErrorDim);
    }

    /*! @return measurement dim */
    int measurementDim() const final {
      return ErrorDim;
    }

    /*! @return The error vector */
    inline const ErrorVectorType& error() const {
      return _e;
    }

    /*! @return The total jacobian */
    inline const TotalJacobianMatrixType& totalJacobian() const {
      return _J;
    }

    /*! Checks if the computation is good for this factor */
    bool isValid() const override;

    /*! @return Information matrix (read only)*/
    inline const InformationMatrixType& informationMatrix() const {
      return _information_matrix;
    }

    /*! Setter for the information matrix
     @param[in] information_matrix inverse covariance matrix to be assigned to the factor
     */
    inline void setInformationMatrix(const InformationMatrixType& information_matrix) {
      _information_matrix = information_matrix;
    }

    /*! Compute error vector and Jacobian
        this function have to be overrided
        in the derived class
        @param[in] error_only true if only the error vector have to be evaluated
    */
    virtual void errorAndJacobian(bool error_only) = 0;

    void compute(bool chi_only = false, bool force = false) override;
    /*! Serialize the measurement contained in the factor */
    void serialize(ObjectData& odata, IdContext& context) override;
    /*! Deserialize the measurement contained in the factor */
    void deserialize(ObjectData& odata, IdContext& context) override;

    /*! Auxiliary function to update the approximate hessian blocks that corresponds
      to the variables involved in the factor */
    template <int r, int c>
    inline void _updateHBlock();

    /*! Auxiliary function to update the gradient vector blocks that corresponds
      to the variables involved in the factor */
    template <int r>
    inline void _updateBBlock();

    /*! Auxiliary function to update the approximate hessian that corresponds
      to the variables involved in the factor */
    inline void updateH();

  protected:
    ErrorVectorType _e;                        /*!< Error vector */
    TotalJacobianMatrixType _J;                /*!< Total jacobian matrix */
    InformationMatrixType _information_matrix; /*!< Information matrix */
    bool _is_valid = true;                     /*! binary flag, true if computation is valid */
  };

} // namespace srrg2_solver
