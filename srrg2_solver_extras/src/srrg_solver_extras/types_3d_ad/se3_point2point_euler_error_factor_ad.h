#pragma once
#include <srrg_pcl/point_types.h>
#include <srrg_solver/solver_core/ad_error_factor.h>
#include <srrg_solver/solver_core/factor_correspondence_driven_dynamic.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h>

namespace srrg2_solver_extras {
  using namespace srrg2_core;
  using namespace srrg2_solver;
  class SE3Point2PointEulerErrorFactorAD : public ADErrorFactor_<3, VariableSE3EulerRightAD> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType          = ADErrorFactor_<3, VariableSE3EulerRightAD>;
    using VariableTupleType = typename BaseType::VariableTupleType;
    using ADErrorVectorType = typename BaseType::ADErrorVectorType;
    using FixedType         = Point3f;
    using MovingType        = Point3f;

    // here we compute our error!
    ADErrorVectorType operator()(VariableTupleType& vars) final {
      Isometry3_<DualValuef> T = vars.at<0>()->adEstimate();
      return T * _moving - _fixed;
    }

    // these convert an arbitrary type to a scalar, for error eval
    void setFixed(const Point3f& fixed_) {
      const Vector3f& fixed_vector = fixed_.coordinates();
      convertMatrix(_fixed, fixed_vector);
    }
    void setMoving(const Point3f& moving_) {
      const Vector3f& moving_vector = moving_.coordinates();
      convertMatrix(_moving, moving_vector);
    }

  protected:
    // parameters
    Vector3_<DualValuef> _fixed;
    Vector3_<DualValuef> _moving;
  };
  using SE3Point2PointEulerErrorFactorCorrespondenceDrivenAD =
    FactorCorrespondenceDrivenDynamic_<SE3Point2PointEulerErrorFactorAD>;
} // namespace srrg2_solver_extras
