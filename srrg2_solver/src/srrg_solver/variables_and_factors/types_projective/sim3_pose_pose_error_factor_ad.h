#pragma once
#include <srrg_solver/solver_core/ad_error_factor.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h>
#include <srrg_solver/variables_and_factors/types_projective/variable_sim3_ad.h>

namespace srrg2_solver {

  using namespace srrg2_core;

  //! @brief pose pose with similiarity error factor ad that uses quaternion vertices
  class Sim3PosePoseErrorFactorAD
    : public ADErrorFactor_<7, VariableSim3QuaternionRightAD, VariableSim3QuaternionRightAD>,
      public MeasurementOwnerEigen_<Similiarity3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BaseType =
      ADErrorFactor_<7, VariableSim3QuaternionRightAD, VariableSim3QuaternionRightAD>;
    using VariableTupleType = typename BaseType::VariableTupleType;
    using ADErrorVectorType = typename BaseType::ADErrorVectorType;
    //! @brief how to compute the error
    ADErrorVectorType operator()(VariableTupleType& vars) override {
      const Similiarity3_<DualValuef>& from = vars.at<0>()->adEstimate();
      const Similiarity3_<DualValuef>& to   = vars.at<1>()->adEstimate();

      Similiarity3_<DualValuef> prediction = from.inverse() * to;

      // ia do the computation (all in dual value)
      return geometry3d::s2v(_inverse_measurement * prediction);
    }

    //! @brief converts the measurement in dual values
    void setMeasurement(const Similiarity3f& iso) override {
      _measurement           = iso;
      Similiarity3f inv_meas = iso.inverse();
      convertMatrix(_inverse_measurement, inv_meas);
    }
    
    void _drawImpl(ViewerCanvasPtr canvas_) const override;

  protected:
    //! @brief measurement
    Similiarity3_<DualValuef> _inverse_measurement;
  };

} // namespace srrg2_solver
