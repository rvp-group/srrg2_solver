#pragma once
#include "ad_variable.h"
#include "error_factor.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  /*! @brief Auto diff Error factor class. It is defined through the same templates as ErrorFactor_.
      In this case you have to specify in the derived class just the () operator, which takes as
      an argument a VariableTupleType. In the operator should compute just the error vector.
      Remember that all the computation has to be performed in DualValuef.
  */
  template <int ErrorDim_, typename... VariableTypes_>
  class ADErrorFactor_ : public ErrorFactor_<ErrorDim_, VariableTypes_...> {
  public:
    using ThisType = ADErrorFactor_<ErrorDim_, VariableTypes_...>;
    using BaseType = ErrorFactor_<ErrorDim_, VariableTypes_...>;
    using VariableTupleType =
      typename BaseType::VariableTupleType; /*!< Extract the variable tuple type */
    static constexpr int NumVariables = BaseType::NumVariables; /*!< Determine the
                                                                  number of variables*/
    static constexpr int ErrorDim = BaseType::ErrorDim;         /*!<Error dimensionality*/
    using ADErrorVectorType       = Eigen::Matrix<DualValuef, ErrorDim, 1>; /*!Error Vector type*/

    template <typename ThisType_, int idx>
    friend struct JUpdater;

    virtual ~ADErrorFactor_() {
    }
    /*! Compute the error vector in DualValuef
      @param[in] vars container of the variables involved in the factor
      @return The error vector
    */
    virtual ADErrorVectorType operator()(VariableTupleType& vars) = 0;

    /*! Compute error vector (call operator()) and determine the Jacobian matrix using the auto diff
     * mechanism */
    void errorAndJacobian(bool error_only = false) final;
    /*! Auxiliary function that extract the idx-th column of the jacobian matrix from the AD error
     * vector */
    template <int idx>
    inline void _updateJacobianAD();
  };
} // namespace srrg2_solver
