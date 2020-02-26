#pragma once
#include "srrg_solver/solver_core/error_factor.h"
#include "srrg_solver/solver_core/factor_correspondence_driven_dynamic.h"
#include "variable_se3.h"
#include <srrg_pcl/point_types.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  class SE3Plane2PlaneErrorFactor : public ErrorFactor_<4, VariableSE3QuaternionRight> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using FixedType  = PointNormal3f;
    using MovingType = PointNormal3f;
    using BaseType = ErrorFactor_<4, VariableSE3QuaternionRight>;
    
    inline void setFixed(const PointNormal3f& fixed_) {
      fixed_point  = &fixed_.coordinates();
      fixed_normal = &fixed_.normal();
    }

    inline void setMoving(const PointNormal3f& moving_) {
      moving_point  = &moving_.coordinates();
      moving_normal = &moving_.normal();
    }
    
    void errorAndJacobian(bool error_only) final;

  protected:
    const Vector3f* moving_point  = nullptr;
    const Vector3f* fixed_point   = nullptr;
    const Vector3f* moving_normal = nullptr;
    const Vector3f* fixed_normal  = nullptr;
  };

  using SE3Plane2PlaneErrorFactorCorrespondenceDriven =
    FactorCorrespondenceDrivenDynamic_<SE3Plane2PlaneErrorFactor>;

} // namespace srrg2_solver
