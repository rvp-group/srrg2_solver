#pragma once
#include "srrg_solver/solver_core/error_factor.h"
#include "srrg_solver/solver_core/factor_correspondence_driven_dynamic.h"
#include "variable_sim3.h"
#include <srrg_pcl/point_types.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  /*! @brief Error factor for Similarity 3D point registration */
  class Sim3Point2PointErrorFactor : public ErrorFactor_<3, VariableSim3QuaternionRight> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using FixedType  = Point3f;
    using MovingType = Point3f;
    using BaseType   = ErrorFactor_<3, VariableSim3QuaternionRight>;

    inline void setFixed(const Point3f& fixed_) {
      fixed = &fixed_.coordinates();
    }
    inline void setMoving(const Point3f& moving_) {
      moving = &moving_.coordinates();
    }

    void errorAndJacobian(bool error_only = false) final;

  protected:
    const Similiarity3f* X = nullptr;
    const Vector3f* moving = nullptr;
    const Vector3f* fixed  = nullptr;
  };

  using Sim3Point2PointErrorFactorCorrespondenceDriven =
    FactorCorrespondenceDrivenDynamic_<Sim3Point2PointErrorFactor>;
} // namespace srrg2_solver
