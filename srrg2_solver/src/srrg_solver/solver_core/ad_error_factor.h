#pragma once
#include "ad_variable.h"
#include "error_factor.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  template <int ErrorDim_, typename... VariableTypes_>
  class ADErrorFactor_ : public ErrorFactor_<ErrorDim_, VariableTypes_...> {
  public:
    using ThisType                    = ADErrorFactor_<ErrorDim_, VariableTypes_...>;
    using BaseType                    = ErrorFactor_<ErrorDim_, VariableTypes_...>;
    using VariableTupleType           = typename BaseType::VariableTupleType;
    static constexpr int NumVariables = BaseType::NumVariables;
    static constexpr int ErrorDim     = BaseType::ErrorDim;
    using ADErrorVectorType           = Eigen::Matrix<DualValuef, ErrorDim, 1>;

    template <typename ThisType_, int idx>
    friend struct JUpdater;

    virtual ~ADErrorFactor_() {
    }

    virtual ADErrorVectorType operator()(VariableTupleType& vars) = 0;

    void errorAndJacobian(bool error_only = false) final;

    template <int idx>
    inline void _updateJacobianAD();
  };
} // namespace srrg2_solver
