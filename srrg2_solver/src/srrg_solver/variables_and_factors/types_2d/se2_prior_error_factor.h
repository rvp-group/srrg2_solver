#pragma once
#include "srrg_solver/solver_core/error_factor.h"
#include "variable_se2.h"
#include <srrg_geometry/geometry2d.h>
#include <srrg_pcl/point_types.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  class SE2PriorErrorFactor : public ErrorFactor_<3, VariableSE2Right>,
                              public MeasurementOwnerEigen_<VariableSE2Right::EstimateType> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseFactorType       = ErrorFactor_<3, VariableSE2Right>;
    using EstimateType         = VariableSE2Right::EstimateType;
    using MeasurementOwnerType = MeasurementOwnerEigen_<EstimateType>;
    void setMeasurement(const MeasurementType& m) override {
      MeasurementOwnerType::setMeasurement(m);
      _inverse_measurement = m.inverse();
    }

    void errorAndJacobian(bool error_only_ = false) final;

  protected:
    MeasurementType _inverse_measurement = MeasurementType::Identity();
  };
} // namespace srrg2_solver
