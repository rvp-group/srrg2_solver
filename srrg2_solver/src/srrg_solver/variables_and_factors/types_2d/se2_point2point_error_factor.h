#pragma once
#include "srrg_solver/solver_core/ad_error_factor.h"
#include "srrg_solver/solver_core/factor_correspondence_driven_dynamic.h"
#include "variable_se2_ad.h"
#include <srrg_geometry/geometry2d.h>
#include <srrg_pcl/point_types.h>

namespace srrg2_solver {
  using namespace srrg2_core;
  // #warning Make a smoking for factor data driven
  /***** point to point *****/

  class SE2Point2PointErrorFactor : public ErrorFactor_<2, VariableSE2Right> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using FixedType  = Point2f;
    using MovingType = Point2f;
    using BaseType   = ErrorFactor_<2, VariableSE2Right>;
    // tg set fix & moving using point types
    inline void setFixed(const Point2f& fixed_) {
      fixed = &fixed_.coordinates();
    }
    inline void setMoving(const Point2f& moving_) {
      moving = &moving_.coordinates();
    }

    void errorAndJacobian(bool error_only = false) final;

  protected:
    const Vector2f* moving = nullptr;
    const Vector2f* fixed  = nullptr;
  };

  // correspondence factor
  using SE2Point2PointErrorFactorCorrespondenceDriven =
    FactorCorrespondenceDrivenDynamic_<SE2Point2PointErrorFactor>;

  class SE2Point2PointWithSensorErrorFactorAD : public ADErrorFactor_<2, VariableSE2RightAD> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using FixedType      = Point2f;
    using MovingType     = Point2f;
    using VariableType   = VariableSE2RightAD;
    using BaseType       = ADErrorFactor_<2, VariableType>;
    using EstimateType   = VariableType::EstimateType;
    using ADEstimateType = VariableType::ADEstimateType;

    // tg set fix & moving using point types
    inline void setFixed(const Point2f& fixed_) {
      _fixed = &fixed_.coordinates();
    }
    inline void setMoving(const Point2f& moving_) {
      _moving = &moving_.coordinates();
    }

    inline void setSensorInRobot(const EstimateType& sensor_in_robot_) {
      convertMatrix(_ad_sensor_in_robot_inverse, sensor_in_robot_.inverse());
    }

    ADErrorVectorType operator()(VariableTupleType& vars) final;

  protected:
    const Vector2f* _moving                    = nullptr;
    const Vector2f* _fixed                     = nullptr;
    ADEstimateType _ad_sensor_in_robot_inverse = ADEstimateType::Identity();
  };

  // correspondence factor
  using SE2Point2PointWithSensorErrorFactorADCorrespondenceDriven =
    FactorCorrespondenceDrivenDynamic_<SE2Point2PointWithSensorErrorFactorAD>;

} // namespace srrg2_solver
