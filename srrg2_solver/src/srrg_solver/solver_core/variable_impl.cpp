#include "variable.h"

namespace srrg2_solver {

  using namespace srrg2_core;

  template <int PerturbationDim_, template <typename Scalar_> typename EstimateType_>
  void VariableGeneric_<PerturbationDim_, EstimateType_>::push() {
    _tainted=true;
    _stack.push_front(_estimate);
  }

  template <int PerturbationDim_, template <typename Scalar_> typename EstimateType_>
  void VariableGeneric_<PerturbationDim_, EstimateType_>::pop() {
    assert(!_stack.empty());
    _tainted=true;
    _estimate = _stack.front();
    _stack.pop_front();
  }

  template <int PerturbationDim_, template <typename Scalar_> typename EstimateType_>
  void VariableGeneric_<PerturbationDim_, EstimateType_>::discardTop() {
    assert(!_stack.empty());
    _stack.pop_front();
  }

  template <int PerturbationDim_, template <typename Scalar_> typename EstimateType_>
  void Variable_<PerturbationDim_, EstimateType_>::serialize(ObjectData& odata,
                                                             IdContext& context) {
    Identifiable::serialize(odata, context);
    odata.setInt("graph_id", this->graphId());
    odata.setInt("status", this->status());
    const int e_rows = this->_estimate.matrix().rows();
    const int e_cols = this->_estimate.matrix().cols();
    ArrayData* adata = new ArrayData;
    for (int r = 0; r < e_rows; ++r) {
      for (int c = 0; c < e_cols; ++c) {
        adata->add(this->_estimate.matrix()(r, c));
      }
    }
    odata.setField("estimate", adata);
  }

  template <int PerturbationDim_, template <typename Scalar_> typename EstimateType_>
  void Variable_<PerturbationDim_, EstimateType_>::deserialize(ObjectData& odata,
                                                               IdContext& context) {
    Identifiable::deserialize(odata, context);
    this->setGraphId(odata.getInt("graph_id"));
    VariableBase::Status st;
    int s = odata.getInt("status");
    switch (s) {
      case VariableBase::Status::Active:
        st = VariableBase::Status::Active;
        break;
      case VariableBase::Status::NonActive:
        st = VariableBase::Status::NonActive;
        break;
      case VariableBase::Status::Fixed:
        st = VariableBase::Status::Fixed;
        break;
      default:
        throw std::runtime_error("Invalid status value in file");
    }
    this->setStatus(st);
    ArrayData* adata = dynamic_cast<ArrayData*>(odata.getField("estimate"));
    int k            = 0;
    const int e_rows = this->_estimate.matrix().rows();
    const int e_cols = this->_estimate.matrix().cols();
    for (int r = 0; r < e_rows; ++r) {
      for (int c = 0; c < e_cols; ++c, ++k) {
        this->_estimate.matrix()(r, c) = (*adata)[k].getFloat();
      }
    }
    this->setEstimate(this->_estimate);
  }

} // namespace srrg2_solver
