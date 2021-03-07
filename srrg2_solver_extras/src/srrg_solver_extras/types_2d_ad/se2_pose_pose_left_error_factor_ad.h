#pragma once
#include <srrg_solver/solver_core/ad_error_factor.h>
#include <srrg_solver/variables_and_factors/types_2d/variable_point2_ad.h>
#include <srrg_solver/variables_and_factors/types_2d/variable_se2_ad.h>

namespace srrg2_solver_extras {

  using namespace srrg2_core;
  using namespace srrg2_solver;

  class SE2PosePoseLeftErrorFactorAD
    : public ADErrorFactor_<3, VariableSE2LeftAD, VariableSE2LeftAD>,
      public MeasurementOwnerEigen_<Isometry2f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BaseType          = ADErrorFactor_<3, VariableSE2LeftAD, VariableSE2LeftAD>;
    using VariableTupleType = typename BaseType::VariableTupleType;
    using MeasurementType   = Isometry2f;
    // we implement an operator that retrieves the value from the
    // var set and computes the error (in dual value format)
    BaseType::ADErrorVectorType operator()(VariableTupleType& vars) {
      //! to retrieve a variable you should know the position in the parameter
      //! pack and the type
      const Isometry2_<DualValuef>& from = vars.at<0>()->adEstimate();
      const Isometry2_<DualValuef>& to   = vars.at<1>()->adEstimate();

      Isometry2_<DualValuef> prediction;
      prediction.translation() =
        from.linear().transpose() * (to.translation() - from.translation());
      prediction.linear() = from.linear().transpose() * to.linear();
      // do the computation (all in dual value)
      return geometry2d::t2v(_inverse_measurement * prediction);
    }
    // define this to set the measurement and make the error function ready
    // this will be visible in the factor!
    void setMeasurement(const Isometry2f& iso) override {
      _measurement        = iso;
      Isometry2f inv_meas = iso.inverse();
      convertMatrix(_inverse_measurement, inv_meas);
    }

    const Isometry2f& measurement() const override {
      return _measurement;
    }
    void _drawImpl(srrg2_core::ViewerCanvasPtr canvas) const override;

  protected:
    // here we store the measurement
    Isometry2_<DualValuef> _inverse_measurement;
  };
} // namespace srrg2_solver_extras
