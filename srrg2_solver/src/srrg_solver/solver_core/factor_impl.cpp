#pragma once
#include "error_factor.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  template <typename VariablePtrTupleType_>
  void Factor_<VariablePtrTupleType_>::pushVariables() {
    for (int i = 0; i < NumVariables; ++i) {
      variable(i)->push();
    }
  }

  template <typename VariablePtrTupleType_>
  void Factor_<VariablePtrTupleType_>::popVariables() {
    for (int i = 0; i < NumVariables; ++i) {
      variable(i)->pop();
    }
  }

  template <typename VariablePtrTupleType_>
  VariableBase::Id Factor_<VariablePtrTupleType_>::variableId(int pos) const {
    return _variables.graphId(pos);
  }

  template <typename VariablePtrTupleType_>
  void Factor_<VariablePtrTupleType_>::setVariableId(int pos,
                                                                        VariableBase::Id id_) {
    _variables.setGraphId(pos, id_);
  }

  template <typename VariablePtrTupleType_>
  void Factor_<VariablePtrTupleType_>::setVariable(int pos, VariableBase* v) {
    _variables.setVariableAt(pos, v);
    Id v_graph_id = v ? v->graphId() : -1;
    _variables.setGraphId(pos, v_graph_id);
  }

  template <typename VariablePtrTupleType_>
  int Factor_<VariablePtrTupleType_>::bind(IdVariablePtrContainer& container) {
    return _variables.bind(container);
  }

  template <typename VariablePtrTupleType_>
  bool Factor_<VariablePtrTupleType_>::isActive() const {
    for (size_t i = 0; i < NumVariables; ++i) {
      if (_b_blocks[i]) {
        return true;
      }
    }
    return false;
  }

  template <typename VariablePtrTupleType_>
  void Factor_<VariablePtrTupleType_>::setHTargetBlock(int r,
                                                            int c,
                                                            MatrixBlockBase* block,
                                                            bool is_transposed) {
    int idx = blockOffset(r, c);
    assert(idx >= 0 && idx < NumHBlocks && "setting a block out of bounds");
    _H_blocks[idx]    = block;
    _H_transpose[idx] = is_transposed;
  }

  template <typename VariablePtrTupleType_>
  void
  Factor_<VariablePtrTupleType_>::setRHSTargetBlock(int r,
                                                         MatrixBlockBase* block) {
    assert(r >= 0 && r < NumVariables);
    _b_blocks[r] = block;
  }

  template <typename VariablePtrTupleType_>
  void Factor_<VariablePtrTupleType_>::clearTargetBlocks() {
    memset(_H_blocks, 0, sizeof(_H_blocks));
    memset(_b_blocks, 0, sizeof(_b_blocks));
    memset(_H_transpose, false, sizeof(_H_transpose));
  }

} // namespace srrg2_solver
