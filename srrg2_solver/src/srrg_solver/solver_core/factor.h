#pragma once
#include "factor_base.h"

namespace srrg2_solver {

  //! @brief middle class for a factor for a  variable problem constructed out of error
  //!        functions (ad and non ad). Derive from this if your measurement is a generic non-Eigen
  //!        Object. you must implement serialization and deserialization
  template <typename VariableTupleType_>
  class Factor_ : public FactorBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using VariableTupleType                   = VariableTupleType_;
    static constexpr int NumVariables         = VariableTupleType::NumVariables;
    static constexpr int TotalPerturbationDim = VariableTupleType::TotalPerturbationDim;
    using TotalPerturbationVectorType         = Eigen::Matrix<float, TotalPerturbationDim, 1>;

    using ThisType                  = Factor_<VariableTupleType>;
    static constexpr int NumBBlocks = VariableTupleType::NumVariables; // for clarity
    static constexpr int NumHBlocks =
      (VariableTupleType::NumVariables + 1) * (VariableTupleType::NumVariables) / 2;

    //! @brief returns the linear address of an h block in the underlying triangular matrix
    static constexpr int blockOffset(int r, int c) {
      return (NumVariables * (NumVariables + 1) / 2) -
             (NumVariables - r + 1) * (NumVariables - r) / 2 + c - r;
    }

    //! @brief same as above, to be used as a function within the non template pain
    template <int r, int c>
    static constexpr int blockOffset() {
      return blockOffset(r, c);
    }

    //! @brief object life - set everything to zero
    Factor_() {
      // we clear the pointers to H and b blocks
      memset(_H_blocks, 0, sizeof(_H_blocks));
      memset(_b_blocks, 0, sizeof(_b_blocks));
      memset(_H_transpose, false, sizeof(_H_transpose));
    }

    virtual ~Factor_() {
    }

    //! @brief push variables in their stacks
    void pushVariables() override;

    //! @brief pop variables from their stacks
    void popVariables() override;

    //! @brief number of variables connected by this factor
    int numVariables() const override {
      return NumVariables;
    }

    //! @brief returns the variable in position pos
    VariableBase::Id variableId(int pos) const override;

    //! @brief changes the variableID of variable in position pos
    void setVariableId(int pos, VariableBase::Id id_) override;

    //! @brief returns the variable at pos
    VariableBase* variable(int pos) override {
      return _variables.variableAt(pos);
    }

    //! @brief returns the variable at pos as const pointer
    const VariableBase* variable(int pos) const override {
      return _variables.variableAt(pos);
    }

    // accessor to the tuple of variable pointers
    inline VariableTupleType& variables() {
      return _variables;
    }

    bool variablesTainted() const override {
      return _variables.tainted();
    }

    //! @brief connects the variables with the corresponding factor.
    //!        For each factor variable, it looks at its id
    //!        and queries the factor container if such a variable is already there.
    //!        If not it creates it.
    int bind(IdVariablePtrContainer& container) override;

    //! @brief checks if this factor is active in the optimization
    bool isActive() const override;

    //! @brief called by solver, it sets the H block for the variable r, c - see FactorBase
    void setHTargetBlock(int r, int c, MatrixBlockBase* block, bool is_transposed) override;

    //! @brief called by solver, it sets the coeff block for the variable r
    void setRHSTargetBlock(int r, MatrixBlockBase* block) override;

    //! @brief invalidates all assignments made to the taeget blocks
    void clearTargetBlocks() override;

  protected:
    //! @brief sets a variable in the error function
    void setVariable(int pos, VariableBase* v) override;

  protected:
    VariableTupleType _variables;
    //! @brief attributes
    MatrixBlockBase* _H_blocks[NumHBlocks];
    bool _H_transpose[NumHBlocks];
    MatrixBlockBase* _b_blocks[NumBBlocks];
  };

} // namespace srrg2_solver
