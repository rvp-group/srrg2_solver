#pragma once
#include <srrg_solver/solver_core/ad_error_factor.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h>

namespace srrg2_solver_extras {
  using namespace srrg2_core;
  using namespace srrg2_solver;
  //! @brief pose pose error factor ad that uses quaternion vertices
  class SE3PosePoseErrorFactorAD:
    public ADErrorFactor_<6, VariableSE3QuaternionRightAD, VariableSE3QuaternionRightAD>,
    public MeasurementOwnerEigen_<Isometry3f> {

    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BaseType =
      ADErrorFactor_<6, VariableSE3QuaternionRightAD, VariableSE3QuaternionRightAD>;
    using VariableTupleType = typename BaseType::VariableTupleType;
    using ADErrorVectorType = typename BaseType::ADErrorVectorType;
    //! @brief how to compute the error
    ADErrorVectorType operator()(VariableTupleType& vars) override{
      const Isometry3_<DualValuef>& from = vars.at<0>()->adEstimate();
      const Isometry3_<DualValuef>& to   = vars.at<1>()->adEstimate();

      Isometry3_<DualValuef> prediction = from.inverse() * to;

      // ia do the computation (all in dual value)
      return geometry3d::t2v(_inverse_measurement * prediction);
    }

    //! @brief converts the measurement in dual values
    void setMeasurement(const Isometry3f& iso) override {
      _measurement        = iso;
      Isometry3f inv_meas = iso.inverse();
      convertMatrix(_inverse_measurement, inv_meas);
    }

    void _drawImpl(ViewerCanvasPtr canvas_) const override;
    protected:
    //! @brief measurement
    Isometry3_<DualValuef> _inverse_measurement;
  };

} // namespace srrg2_solver
