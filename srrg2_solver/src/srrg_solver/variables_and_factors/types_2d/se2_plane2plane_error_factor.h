#pragma once
#include "srrg_solver/solver_core/ad_error_factor.h"
#include "srrg_solver/solver_core/factor_correspondence_driven_dynamic.h"
#include "variable_se2_ad.h"
#include <srrg_geometry/geometry2d.h>
#include <srrg_pcl/point_types.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  class SE2Plane2PlaneErrorFactor : public ErrorFactor_<3, VariableSE2Right> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using FixedType  = PointNormal2f;
    using MovingType = PointNormal2f;
    using BaseType   = ErrorFactor_<3, VariableSE2Right>;

    inline void setFixed(const PointNormal2f& fixed_) {
      _fixed_point  = &fixed_.coordinates();
      _fixed_normal = &fixed_.normal();
    }
    inline void setMoving(const PointNormal2f& moving_) {
      _moving_point  = &moving_.coordinates();
      _moving_normal = &moving_.normal();
    }

    void errorAndJacobian(bool error_only = false) final;

  protected:
    const Vector2f* _moving_point  = nullptr;
    const Vector2f* _moving_normal = nullptr;
    const Vector2f* _fixed_point   = nullptr;
    const Vector2f* _fixed_normal  = nullptr;
  };
  // correspondence factor
  using SE2Plane2PlaneErrorFactorCorrespondenceDriven =
    FactorCorrespondenceDrivenDynamic_<SE2Plane2PlaneErrorFactor>;

  class SE2Plane2PlaneWithSensorErrorFactorAD : public ADErrorFactor_<3, VariableSE2RightAD> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using FixedType    = PointNormal2f;
    using MovingType   = PointNormal2f;
    using VariableType = VariableSE2RightAD;
    using BaseType     = ADErrorFactor_<2, VariableType>;

    using EstimateType   = VariableType::EstimateType;
    using ADEstimateType = VariableType::ADEstimateType;

    inline void setFixed(const PointNormal2f& fixed_) {
      _fixed_point  = &fixed_.coordinates();
      _fixed_normal = &fixed_.normal();
    }
    inline void setMoving(const PointNormal2f& moving_) {
      _moving_point  = &moving_.coordinates();
      _moving_normal = &moving_.normal();
    }

    inline void setSensorInRobot(const EstimateType& sensor_in_robot_) {
      Isometry2f isr = sensor_in_robot_.inverse();
      convertMatrix(_ad_robot_in_sensor, isr);
    }

    ADErrorVectorType operator()(VariableTupleType& vars) final;

  protected:
    const Vector2f* _moving_point      = nullptr;
    const Vector2f* _moving_normal     = nullptr;
    const Vector2f* _fixed_point       = nullptr;
    const Vector2f* _fixed_normal      = nullptr;
    ADEstimateType _ad_robot_in_sensor = ADEstimateType::Identity();
  };
  // correspondence factor
  using SE2Plane2PlaneWithSensorErrorFactorADCorrespondenceDriven =
    FactorCorrespondenceDrivenDynamic_<SE2Plane2PlaneWithSensorErrorFactorAD>;

} // namespace srrg2_solver
