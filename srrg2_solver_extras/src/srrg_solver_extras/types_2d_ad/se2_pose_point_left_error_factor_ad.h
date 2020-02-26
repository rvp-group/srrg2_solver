#pragma once
#include <srrg_solver/solver_core/ad_error_factor.h>
#include <srrg_solver/variables_and_factors/types_2d/variable_point2_ad.h>
#include <srrg_solver/variables_and_factors/types_2d/variable_se2_ad.h>

namespace srrg2_solver_extras {
  using namespace srrg2_core;
  using namespace srrg2_solver;
  // pose-point error functor, se2 AD
  class SE2PosePointLeftErrorFactorAD
    : public ADErrorFactor_<2, VariableSE2LeftAD, VariablePoint2AD>,
      public MeasurementOwnerEigen_<Vector2f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType          = ADErrorFactor_<2, VariableSE2LeftAD, VariablePoint2AD>;
    using VariableTupleType = BaseType::VariableTupleType;

    BaseType::ADErrorVectorType operator()(VariableTupleType& vars) final {
      //! to retrieve a variable you should know the position in the parameter pack
      //! and the type
      const Isometry2_<DualValuef>& from = vars.at<0>()->adEstimate();
      const Vector2_<DualValuef>& to     = vars.at<1>()->adEstimate();

      // do the computation (all in dual value)
      return from.inverse() * to - _ad_measurement;
    }

    // define this to set the measurement and make the error function ready
    // this will be visible in the factor!
    void setMeasurement(const Vector2f& m) override {
      _measurement = m;
      convertMatrix(_ad_measurement, _measurement);
    }

  protected:
    Vector2_<DualValuef> _ad_measurement;
  };
} // namespace srrg2_solver_extras
