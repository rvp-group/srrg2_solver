#pragma once

#include "srrg_solver/solver_core/error_factor.h"
#include "variable_se3.h"

namespace srrg2_solver {

  /**
   * @brief Standard SE3 PGO factor.
   * Error vector is composed like [x y z qx qy qz]
   */
  class SE3PosePoseGeodesicErrorFactor
    : public ErrorFactor_<6, VariableSE3QuaternionRight, VariableSE3QuaternionRight>,
      public MeasurementOwnerEigen_<Isometry3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = ErrorFactor_<6, VariableSE3QuaternionRight, VariableSE3QuaternionRight>;
    using Scalar   = MeasurementType::Scalar;
    using MeasurementOwnerType = MeasurementOwnerEigen_<Isometry3f>;

    void setMeasurement(const MeasurementType& measurement_) override {
      MeasurementOwnerType::setMeasurement(measurement_);
      _inverse_measurement = _measurement.inverse();
    }

    void errorAndJacobian(bool error_only_ = false) override;

    void _drawImpl(ViewerCanvasPtr canvas_) const;

  protected:
    MeasurementType _inverse_measurement = MeasurementType::Identity();
  };
} // namespace srrg2_solver
