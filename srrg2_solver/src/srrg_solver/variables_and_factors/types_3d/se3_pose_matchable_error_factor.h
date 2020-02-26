#pragma once
#include "srrg_solver/solver_core/error_factor.h"
#include "variable_matchable.h"
#include "variable_se3.h"

namespace srrg2_solver {
  class SE3PoseMatchableEulerLeftErrorFactor
    : public ErrorFactor_<7, VariableSE3EulerLeft, VariableMatchable>,
      public MeasurementOwner_<Matchablef> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = ErrorFactor_<7, VariableSE3EulerLeft, VariableMatchable>;

    // @brief tg this methods became aliases for serialize/deserialize due to the fact
    // that Matchablef is a non-Eigen type 
    void serializeMeasurement(ObjectData& odata, IdContext& context) final;
    void deserializeMeasurement(ObjectData& odata, IdContext& context) final;
    void draw(ViewerCanvasPtr canvas_) const override;
    void errorAndJacobian(bool error_only_ = false) final;
  };

} // namespace srrg2_solver
