#pragma once
#include "measurement_owner.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  template <typename MeasurementType_>
  void MeasurementOwnerEigen_<MeasurementType_>::serializeMeasurement(ObjectData& odata,
                                                                      IdContext& context) {
    int m_rows = this->_measurement.matrix().rows();
    int m_cols = this->_measurement.matrix().cols();
    if (!m_rows || !m_cols) {
      return;
    }
    odata.setEigen<MeasurementType_>("measurement", this->_measurement);

    // ia old serialization of eigen data
    //    ArrayData* mdata = new ArrayData;
    //    for (int r = 0; r < m_rows; ++r) {
    //      for (int c = 0; c < m_cols; ++c) {
    //        mdata->add(this->_measurement.matrix()(r, c));
    //      }
    //    }
    //    odata.setField("measurement", mdata);
  }

  template <typename MeasurementType_>
  void MeasurementOwnerEigen_<MeasurementType_>::deserializeMeasurement(ObjectData& odata,
                                                                        IdContext& context) {
    int m_rows = this->_measurement.matrix().rows();
    int m_cols = this->_measurement.matrix().cols();
    if (!m_rows || !m_cols)
      return;

    MeasurementType_ meas = odata.getEigen<MeasurementType_>("measurement");

    // ia old serialization of eigen data
    // ArrayData* mdata = dynamic_cast<ArrayData*>(odata.getField("measurement"));
    // int k            = 0;
    // for (int r = 0; r < m_rows; ++r) {
    //   for (int c = 0; c < m_cols; ++c, ++k) {
    //     this->_measurement.matrix()(r, c) = (*mdata)[k].getFloat();
    //   }
    // }

    this->setMeasurement(meas);
  }

} // namespace srrg2_solver
