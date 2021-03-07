#pragma once

#include <srrg_solver/solver_core/ad_error_factor.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h>

namespace srrg2_solver_extras {
  using namespace srrg2_core;
  using namespace srrg2_solver;
  //! @brief tg chordal factor with autodiff and quaternion right
  class SE3PosePoseChordalQuaternionErrorFactorAD
    : public ADErrorFactor_<12, VariableSE3QuaternionRightAD, VariableSE3QuaternionRightAD>,
      public MeasurementOwnerEigen_<Isometry3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BaseFunctorType =
      ADErrorFactor_<12, VariableSE3QuaternionRightAD, VariableSE3QuaternionRightAD>;
    using VariableTupleType = typename BaseFunctorType::VariableTupleType;

    //! @brief compute the error as quaternion
    BaseFunctorType::ADErrorVectorType operator()(VariableTupleType& vars) final {
      const Isometry3_<DualValuef>& from = vars.at<0>()->adEstimate();
      const Isometry3_<DualValuef>& to   = vars.at<1>()->adEstimate();

      Isometry3_<DualValuef> prediction = from.inverse() * to;
      Isometry3_<DualValuef> error      = Isometry3_<DualValuef>::Identity();
      error.matrix()                    = prediction.matrix() - _ad_measurement.matrix();

      return geometry3d::flattenByCols(error);
    }

    // define this to set the measurement and make the error function ready
    // this will be visible in the factor!
    void setMeasurement(const Isometry3f& iso) override {
      _measurement = iso;
      convertMatrix(_ad_measurement, iso);
    }

    void _drawImpl(ViewerCanvasPtr canvas_) const override;

  protected:
    // here we store the measurement
    Isometry3_<DualValuef> _ad_measurement = Isometry3_<DualValuef>::Identity();
    Isometry3f _measurement                = Isometry3f::Identity();
  };
} // namespace srrg2_solver_extras
