#pragma once
#include <srrg_solver/solver_core/ad_error_factor.h>
#include <srrg_solver/variables_and_factors/types_2d/variable_point2_ad.h>
#include <srrg_solver/variables_and_factors/types_2d/variable_se2_ad.h>

namespace srrg2_solver_extras {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  // pose-point bearing error functor, se2 AD
  class SE2PosePointBearingLeftErrorFactorAD
    : public ADErrorFactor_<1, VariableSE2LeftAD, VariablePoint2AD>,
      public MeasurementOwnerEigen_<Vector1f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType          = ADErrorFactor_<1, VariableSE2LeftAD, VariablePoint2AD>;
    using VariableTupleType = BaseType::VariableTupleType;

    inline BaseType::ADErrorVectorType operator()(VariableTupleType& vars) final {
      //! to retrieve a variable you should know the position in the parameter pack
      //! and the type
      const Isometry2_<DualValuef>& from = vars.at<0>()->adEstimate();
      const Vector2_<DualValuef>& to     = vars.at<1>()->adEstimate();

      // do the computation (all in dual value)
      Vector2_<DualValuef> local_point = from.inverse() * to;
      DualValuef e = atan2(local_point.y(), local_point.x()) - _ad_measurement(0);
      e            = atan2(sin(e), cos(e));

      ADErrorVectorType returned;
      returned(0) = e;
      return returned;
    }

    // define this to set the measurement and make the error function ready
    // this will be visible in the factor!
    void setMeasurement(const Vector1f& m) override {
      this->_measurement = m;
      convertMatrix(_ad_measurement, this->_measurement);
    }

    const Vector1f& measurement() const override {
      return _measurement;
    }

  protected:
    Vector1_<DualValuef> _ad_measurement;
  };
} // namespace srrg2_solver_extras
