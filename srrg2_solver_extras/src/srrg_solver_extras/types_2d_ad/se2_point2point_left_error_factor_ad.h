#pragma once
#include <srrg_geometry/geometry2d.h>
#include <srrg_pcl/point_types.h>
#include <srrg_solver/solver_core/factor_correspondence_driven_dynamic.h>
#include <srrg_solver/solver_core/ad_error_factor.h>
#include <srrg_solver/variables_and_factors/types_2d/variable_se2_ad.h>

namespace srrg2_solver_extras {
  using namespace srrg2_core;
  using namespace srrg2_solver;
  
  class SE2Point2PointLeftErrorFactorAD : public ADErrorFactor_<2, VariableSE2LeftAD> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType          = ADErrorFactor_<2, VariableSE2LeftAD>;
    using VariableTupleType = typename BaseType::VariableTupleType;
    using FixedType         = Point2f;
    using MovingType        = Point2f;

    // here we compute our error!
    BaseType::ADErrorVectorType operator()(VariableTupleType& vars) final {
      const Isometry2_<DualValuef>& T = vars.at<0>()->adEstimate();
      return T * _moving - _fixed;
    }

    // these convert an arbitrary type to a scalar, for error eval
    void setFixed(const Point2f& fixed_) {
      convertMatrix(_fixed, fixed_.coordinates());
    }
    void setMoving(const Point2f& moving_) {
      convertMatrix(_moving, moving_.coordinates());
    }

  protected:
    // parameters
    Vector2_<DualValuef> _fixed;
    Vector2_<DualValuef> _moving;
  };

  // correspondence factor
  using SE2Point2PointLeftErrorFactorCorrespondenceDrivenAD =
    FactorCorrespondenceDrivenDynamic_<SE2Point2PointLeftErrorFactorAD>;
} // namespace srrg2_solver_extras
