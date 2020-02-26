#include "error_factor.h"
#include "factor_impl.cpp"
#include "measurement_owner_impl.cpp"

namespace srrg2_solver {
  using namespace srrg2_core;

  // factor for a  variable problem  constructed out of error functions (ad and non ad)
  // usage:
  //   using MyErrorFactor=ErrorFactor_<MyErrorFunction>;

  // Ugly eh :))
  template <typename ThisType, int r, int c>
  struct ColUpdater {
    static inline void update(ThisType& factor) {
      factor.template _updateHBlock<r, c>();
      ColUpdater<ThisType, r, c - 1>::update(factor);
    }
  };

  template <typename ThisType, int r>
  struct ColUpdater<ThisType, r, r> {
    static inline void update(ThisType& factor) {
      factor.template _updateHBlock<r, r>();
    }
  };

  template <typename ThisType, int r, int c>
  struct RowUpdater {
    static inline void update(ThisType& factor) {
      ColUpdater<ThisType, r, c>::update(factor);
      RowUpdater<ThisType, r - 1, c>::update(factor);
      factor.template _updateBBlock<r>();
    }
  };

  template <typename ThisType, int c>
  struct RowUpdater<ThisType, 0, c> {
    static inline void update(ThisType& factor) {
      ColUpdater<ThisType, 0, c>::update(factor);
      factor.template _updateBBlock<0>();
    }
  };

  template <int ErrorDim_, typename... VariableTypes_>
  bool ErrorFactor_<ErrorDim_, VariableTypes_...>::isValid() const {
    return _is_valid;
  }

  template <int ErrorDim_, typename... VariableTypes_>
  void ErrorFactor_<ErrorDim_, VariableTypes_...>::compute(bool chi_only, bool force) {
    this->_stats.status = FactorStats::Status::Suppressed;
    this->_stats.chi    = 0;

    if (!this->isActive() && !force) {
      return;
    }

    errorAndJacobian(chi_only);
    if (!isValid()) {
      return;
    }

    const ErrorVectorType& error          = this->error();
    const ErrorVectorType& weighted_error = this->informationMatrix() * error;
    this->_stats.chi                      = error.transpose() * weighted_error;

    this->robustify();
    if (chi_only) {
      return;
    }
    // std::cerr << _J << std::endl;
    updateH();
  }

  template <int ErrorDim_, typename... VariableTypes_>
  template <int r, int c>
  void ErrorFactor_<ErrorDim_, VariableTypes_...>::_updateHBlock() {
    const int BlockRows = ThisType::template perturbationDim<r>();
    const int BlockCols = ThisType::template perturbationDim<c>();
    int idx             = BaseType::template blockOffset<r, c>();
    // a block is null when a variable is fixed
    if (!this->_H_blocks[idx]) {
      return;
    }
    if (!this->_H_transpose[idx]) {
      assert(this->_H_blocks[idx]->rows() == BlockRows &&
             this->_H_blocks[idx]->cols() == BlockCols && "dimension mismatch");
      Eigen::Map<Eigen::Matrix<float, BlockRows, BlockCols>> target_H(
        this->_H_blocks[idx]->storage());
      target_H.noalias() += this->_kernel_scales[1] * ThisType::template jacobian<r>().transpose() *
                            _information_matrix * ThisType::template jacobian<c>();

    } else {
      assert(this->_H_blocks[idx]->rows() == BlockCols &&
             this->_H_blocks[idx]->cols() == BlockRows && "dimension mismatch");

      Eigen::Map<Eigen::Matrix<float, BlockCols, BlockRows>> target_H(
        this->_H_blocks[idx]->storage());
      target_H.noalias() += this->_kernel_scales[1] * ThisType::template jacobian<c>().transpose() *
                            _information_matrix * ThisType::template jacobian<r>();
    }
  }

  template <int ErrorDim_, typename... VariableTypes_>
  template <int r>
  void ErrorFactor_<ErrorDim_, VariableTypes_...>::_updateBBlock() {
    const int BlockRows = ThisType::template perturbationDim<r>();
    using namespace std;
    if (!this->_b_blocks[r]) {
      // a block is null if the variable is fixed
      return;
    }
    Eigen::Map<Eigen::Matrix<float, BlockRows, 1>> target_b(this->_b_blocks[r]->storage());
    assert(this->_b_blocks[r]->rows() == BlockRows && "dimension mismatch");
    target_b.noalias() -= this->_kernel_scales[1] * ThisType::template jacobian<r>().transpose() *
                          ThisType::informationMatrix() * ThisType::error();
  }

  template <int ErrorDim_, typename... VariableTypes_>
  void ErrorFactor_<ErrorDim_, VariableTypes_...>::updateH() {
    RowUpdater<ThisType, NumVariables - 1, NumVariables - 1>::update(*this);
  }

  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  // ------------------------------------------------------------------------------------------- //
  template <int ErrorDim_, typename... VariableTypes_>
  void ErrorFactor_<ErrorDim_, VariableTypes_...>::serialize(ObjectData& odata,
                                                             IdContext& context) {
    Identifiable::serialize(odata, context);
    odata.setInt("graph_id", BaseType::graphId());
    odata.setBool("enabled", BaseType::enabled());

    ArrayData* adata = new ArrayData;
    for (int pos = 0; pos < NumVariables; ++pos) {
      adata->add((int) BaseType::variableId(pos));
    }
    odata.setField("variables", adata);

    MeasurementOwnerBase* m_owner = dynamic_cast<MeasurementOwnerBase*>(this);
    if (m_owner) {
      m_owner->serializeMeasurement(odata, context);
    }

    int i_rows       = _information_matrix.rows();
    int i_cols       = _information_matrix.cols();
    ArrayData* idata = new ArrayData;
    for (int r = 0; r < i_rows; ++r) {
      for (int c = r; c < i_cols; ++c) {
        idata->add(_information_matrix(r, c));
      }
    }
    odata.setField("information", idata);
  }

  template <int ErrorDim_, typename... VariableTypes_>
  void ErrorFactor_<ErrorDim_, VariableTypes_...>::deserialize(ObjectData& odata,
                                                               IdContext& context) {
    Identifiable::deserialize(odata, context);
    FactorBase::_graph_id = odata.getInt("graph_id");
    if (odata.getField("enabled")) {
      FactorBase::_enabled = odata.getBool("enabled");
    }
    ArrayData* adata = dynamic_cast<ArrayData*>(odata.getField("variables"));
    int pos          = 0;
    for (auto it = adata->begin(); it != adata->end(); ++it) {
      ThisType::_variables.setGraphId(pos, (*it)->getInt());
      ++pos;
    }

    MeasurementOwnerBase* m_owner = dynamic_cast<MeasurementOwnerBase*>(this);
    if (m_owner) {
      m_owner->deserializeMeasurement(odata, context);
    }

    int i_rows       = _information_matrix.rows();
    int i_cols       = _information_matrix.cols();
    ArrayData* idata = dynamic_cast<ArrayData*>(odata.getField("information"));
    int k            = 0;
    for (int r = 0; r < i_rows; ++r) {
      for (int c = r; c < i_cols; ++c, ++k) {
        _information_matrix(r, c) = (*idata)[k].getFloat();
        _information_matrix(c, r) = _information_matrix(r, c);
      }
    }
  }

} // namespace srrg2_solver
