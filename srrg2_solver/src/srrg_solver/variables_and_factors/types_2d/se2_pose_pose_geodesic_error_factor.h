#pragma once
#include "srrg_solver/solver_core/error_factor.h"
#include "variable_se2.h"
#include <srrg_geometry/geometry2d.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  /** @brief 2D PGO error factor exploiting geodesic distance.
   * Estimate poses of two variables given their measured relative pose.
   */
  class SE2PosePoseGeodesicErrorFactor : public ErrorFactor_<3, VariableSE2Right, VariableSE2Right>,
                                         public MeasurementOwnerEigen_<Isometry2f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseFactorType       = ErrorFactor_<3, VariableSE2Right, VariableSE2Right>;
    using EstimateType         = VariableSE2Right::EstimateType;
    using MeasurementOwnerType = MeasurementOwnerEigen_<EstimateType>;

    void setMeasurement(const MeasurementType& m) override {
      MeasurementOwnerType::setMeasurement(m);
      _inverse_measured_relative_pose = MeasurementOwnerType::_measurement.inverse();
    }

    void errorAndJacobian(bool error_only_ = false) final;
    void _drawImpl(srrg2_core::ViewerCanvasPtr canvas) const override;

  protected:
    MeasurementType _inverse_measured_relative_pose; /**< Cache the inverse measurement */
  };
} // namespace srrg2_solver
