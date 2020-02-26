#pragma once

#include "srrg_solver/solver_core/error_factor.h"
#include "variable_se3.h"

namespace srrg2_solver {

  //! @brief error function for standard SE3 PGO. Error vector is [x y z qx qy qz]
  class SE3PosePoseGeodesicErrorFactor
    : public ErrorFactor_<6, VariableSE3QuaternionRight, VariableSE3QuaternionRight>,
      public MeasurementOwnerEigen_<Isometry3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // ia some usings
    using BaseType = ErrorFactor_<6, VariableSE3QuaternionRight, VariableSE3QuaternionRight>;
    using Scalar   = MeasurementType::Scalar;

    //! @brief set measurement from isometry3
    void setMeasurement(const MeasurementType& m_) override {
      this->_measurement   = m_;
      _inverse_measurement = _measurement.inverse();
    }

    //! @brief compute error and - eventually - jacobian
    void errorAndJacobian(bool error_only_ = false) final;

    void draw(ViewerCanvasPtr canvas_) const final;

  protected:
    MeasurementType _inverse_measurement;
  };
} // namespace srrg2_solver
