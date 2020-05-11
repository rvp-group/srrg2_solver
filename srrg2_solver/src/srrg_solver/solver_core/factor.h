#pragma once
#include "factor_base.h"

namespace srrg2_solver {
  /*! @brief Factor that don't specify how it contribute to the approximate hessian and gradient
    vector */
  /*! Middle class for a factor. Derive from this if your measurement is a generic non-Eigen
      Object, or you want to exploit some special structure of the approximate hessian matrix block
      computation or Jacobian structure. In the derived class you must implement:

      - The serialize()/deserialize() functions.
      - compute() which should fill the approximate hessian matrix/gradient blocks that correspond
        to the variables involved

      The Factor_ is instantiated on a VariableTupleType_, which is a VariablePtrTuple_ that contain
      the types of all the variables involved in the factor.
  */
  template <typename VariableTupleType_>
  class Factor_ : public FactorBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using VariableTupleType           = VariableTupleType_;
    static constexpr int NumVariables = VariableTupleType::NumVariables; /*!<Extract the
                                         number of variables from VariableTupleType_ */
    static constexpr int TotalPerturbationDim = VariableTupleType::TotalPerturbationDim; /*!<
                                                 Extract
                                                 the total dimension
                                                 of the perturbation vector
                                                 of the variables involved in the factor */
    using TotalPerturbationVectorType = Eigen::Matrix<float, TotalPerturbationDim, 1>;

    using ThisType                  = Factor_<VariableTupleType>;
    static constexpr int NumBBlocks = NumVariables; /*!< Extract
                                          number of
                                          gradient vector
                                          blocks
                                          (one for variable) */
    static constexpr int NumHBlocks = (NumVariables + 1) * (NumVariables) / 2;
    /*!<
         Number of approx
         hessian blocks
         (the matrix is symmetric)
       */

    /*! Returns the linear address of an H block in the underlying triangular matrix
      @param[in] r row block index
      @param[in] c col block index
      @return block index in the linear structure
    */
    static constexpr int blockOffset(int r, int c) {
      return (NumVariables * (NumVariables + 1) / 2) -
             (NumVariables - r + 1) * (NumVariables - r) / 2 + c - r;
    }

    /*! Returns the linear address of an H block in the underlying triangular matrix
      - same as blockOffset(int r, int c) using templates

      @return block index in the linear structure */
    template <int r, int c>
    static constexpr int blockOffset() {
      return blockOffset(r, c);
    }

    /*! object life - set all the H and b blocks addresses to zero */
    Factor_() {
      memset(_H_blocks, 0, sizeof(_H_blocks));
      memset(_b_blocks, 0, sizeof(_b_blocks));
      memset(_H_transpose, false, sizeof(_H_transpose));
    }

    virtual ~Factor_() {
    }

    // push variables in their stacks
    void pushVariables() override;

    // pop variables from their stacks
    void popVariables() override;

    /*! @return number of variables connected by this factor */
    int numVariables() const override {
      return NumVariables;
    }

    /*! @return the variable in position pos */
    VariableBase::Id variableId(int pos) const override;

    /*! Changes the variable id of variable in position pos
      @param[in] pos variable index in VariableTupleType_
      @param[in] id_ variable graph id
    */
    void setVariableId(int pos, VariableBase::Id id_) override;

    /*! Get the variable pointer at position pos
      @param[in] pos index of the variable in the VariableTupleType_
      @return variable pointer
    */
    VariableBase* variable(int pos) override {
      return _variables.variableAt(pos);
    }

    /*!
      @param[in] pos index of the variable in the VariableTupleType_
      @return variable pointer (const)
    */
    const VariableBase* variable(int pos) const override {
      return _variables.variableAt(pos);
    }

    /*! Accessor to the tuple of variable pointers*/
    inline VariableTupleType& variables() {
      return _variables;
    }

    /* taint all the variables in the tuple */
    bool variablesTainted() const override {
      return _variables.tainted();
    }

    /*! It connect the variables to this factor - see FactorBase */
    int bind(IdVariablePtrContainer& container) override;

    /*! Checks if this factor is active in the optimization */
    bool isActive() const override;

    /*! It sets the H block for the variable r, c - see FactorBase */
    void setHTargetBlock(int r, int c, MatrixBlockBase* block, bool is_transposed) override;

    /*! It sets the B block for the variable r - see FactorBase */
    void setRHSTargetBlock(int r, MatrixBlockBase* block) override;

    /*! Invalidates all assignments made to the target blocks - see FactorBase */
    void clearTargetBlocks() override;

  protected:
    /*! Sets a variable in the factor
      @param[in] pos index in VariableTupleType_
      @param[in] v variable pointer to be inserted
    */
    void setVariable(int pos, VariableBase* v) override;

  protected:
    VariableTupleType _variables; /*!< Variable container of the factor, see VariablePtrTuple_*/
    MatrixBlockBase* _H_blocks[NumHBlocks]; /*!< Approximate hessian block pointers */
    bool _H_transpose[NumHBlocks]; /*< boolean array which tells which blocks of H have to be
                                      transposed */
    MatrixBlockBase* _b_blocks[NumBBlocks]; /*!< Approximate hessian block pointers */
  };

} // namespace srrg2_solver
