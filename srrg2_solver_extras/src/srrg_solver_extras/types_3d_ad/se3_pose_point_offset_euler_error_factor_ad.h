#pragma once

#include <srrg_solver/solver_core/ad_error_factor.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h>

namespace srrg2_solver_extras {
  using namespace srrg2_core;
  using namespace srrg2_solver;
  // tg pose-point error functor with offset, se3 AD
  class SE3PosePointOffsetEulerErrorFactorAD
    : public ADErrorFactor_<3, VariableSE3EulerRightAD, VariablePoint3AD, VariableSE3EulerRightAD>,
      public MeasurementOwnerEigen_<Vector3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType =
      ADErrorFactor_<3, VariableSE3EulerRightAD, VariablePoint3AD, VariableSE3EulerRightAD>;
    using VariableTupleType = typename BaseType::VariableTupleType;
    using ADErrorVectorType = typename BaseType::ADErrorVectorType;
    ADErrorVectorType operator()(VariableTupleType& vars) final {
      //! to retrieve a variable you should know the position in the parameter pack
      //! and the type
      const Isometry3_<DualValuef>& from   = vars.at<0>()->adEstimate();
      const Vector3_<DualValuef>& to       = vars.at<1>()->adEstimate();
      const Isometry3_<DualValuef>& offset = vars.at<2>()->adEstimate();

      Isometry3_<DualValuef> inv_sensor_pose = (from * offset).inverse();
      // do the computation (all in dual value)
      return inv_sensor_pose * to - _ad_measurement;
    }

    // define this to set the measurement and make the error function ready
    // this will be visible in the factor!
    void setMeasurement(const Vector3f& m) override {
      _measurement = m;
      convertMatrix(_ad_measurement, _measurement);
    }

  protected:
    Vector3_<DualValuef> _ad_measurement;
  };
} // namespace srrg2_solver_extras
