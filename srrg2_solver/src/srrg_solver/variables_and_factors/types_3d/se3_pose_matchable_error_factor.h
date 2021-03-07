#pragma once
#include "srrg_solver/solver_core/error_factor.h"
#include "variable_matchable.h"
#include "variable_se3.h"

namespace srrg2_solver {

  /**
   * @brief SE3-Machable error factor, used in global optimization
   * Error is a 7D vector composed as [e_origin e_direction e_orthogonality]
   * The SE3 variable required is Euler-Left; the measurement is a non-Eigen type
   */
  class SE3PoseMatchableEulerLeftErrorFactor
    : public ErrorFactor_<7, VariableSE3EulerLeft, VariableMatchable>,
      public MeasurementOwner_<Matchablef> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = ErrorFactor_<7, VariableSE3EulerLeft, VariableMatchable>;

    /**
     *  @brief non-Eigen measurement requires custom serialization of the measurement
     */
    void serializeMeasurement(ObjectData& odata, IdContext& context) final;

    /**
     *  @brief non-Eigen measurement requires custom deserialization of the measurement
     */
    void deserializeMeasurement(ObjectData& odata, IdContext& context) final;

    void _drawImpl(ViewerCanvasPtr canvas_) const override;
    void errorAndJacobian(bool error_only_ = false) final;
  };

} // namespace srrg2_solver
