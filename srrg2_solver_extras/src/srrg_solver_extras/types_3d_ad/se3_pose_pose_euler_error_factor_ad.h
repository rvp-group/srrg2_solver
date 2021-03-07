#pragma once
#include <srrg_solver/solver_core/ad_error_factor.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h>

namespace srrg2_solver_extras {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  //! 1. we construct a functor representing our error function
  //! deriving it from ADMultiErrorFunctor_<ErrorDim, Variables...>
  //! Variables is a list of all variables we use in the computation

  class SE3PosePoseEulerErrorFactorAD
    : public ADErrorFactor_<6, VariableSE3EulerRightAD, VariableSE3EulerRightAD>,
      public MeasurementOwnerEigen_<Isometry3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BaseType          = ADErrorFactor_<6, VariableSE3EulerRightAD, VariableSE3EulerRightAD>;
    using VariableTupleType = typename BaseType::VariableTupleType;

    // we implement an operator that retrieves the value from the
    // var set and computes the error (in dual value format)
    BaseType::ADErrorVectorType operator()(VariableTupleType& vars) final {
      //! to retrieve a variable you should know the position in the parameter pack
      //! and the type
      const Isometry3_<DualValuef>& from = vars.at<0>()->adEstimate();
      const Isometry3_<DualValuef>& to   = vars.at<1>()->adEstimate();
      Isometry3_<DualValuef> prediction  = from.inverse() * to;
      return geometry3d::t2v(_inverse_measurement * prediction);
    }

    // define this to set the measurement and make the error function ready
    // this will be visible in the factor!
    void setMeasurement(const Isometry3f& iso) override {
      _measurement        = iso;
      Isometry3f inv_meas = iso.inverse();
      convertMatrix(_inverse_measurement, inv_meas);
    }

    void _drawImpl(ViewerCanvasPtr canvas_) const override;

  protected:
    // here we store the measurement
    Isometry3_<DualValuef> _inverse_measurement;
  };
} // namespace srrg2_solver_extras
