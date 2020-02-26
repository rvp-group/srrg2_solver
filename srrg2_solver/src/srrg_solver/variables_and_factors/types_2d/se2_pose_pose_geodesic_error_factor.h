#pragma once
#include "srrg_solver/solver_core/error_factor.h"
#include "variable_se2.h"
#include <srrg_geometry/geometry2d.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  class SE2PosePoseGeodesicErrorFactor : public ErrorFactor_<3, VariableSE2Right, VariableSE2Right>,
                                         public MeasurementOwnerEigen_<Isometry2f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    void setMeasurement(const MeasurementType& m) override {
      this->_measurement              = m;
      _inverse_measured_relative_pose = m.inverse();
    }

    void errorAndJacobian(bool error_only_ = false) final;
    void draw(srrg2_core::ViewerCanvasPtr canvas) const final;

  protected:
    MeasurementType _inverse_measured_relative_pose;
  };
} // namespace srrg2_solver
