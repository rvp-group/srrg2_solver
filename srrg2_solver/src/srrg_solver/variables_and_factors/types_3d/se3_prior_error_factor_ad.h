#pragma once
#include "srrg_solver/solver_core/ad_error_factor.h"
#include "srrg_solver/solver_core/measurement_owner.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  /** @brief 3D Pose prior factor with autodiff.
   * Creates a unary factor to make converge the estimated pose using the measurement.
   */
  class SE3PriorErrorFactorAD
    : public ADErrorFactor_<6, VariableSE3QuaternionRightAD>,
      public MeasurementOwnerEigen_<VariableSE3QuaternionRightAD::EstimateType> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType             = ADErrorFactor_<6, VariableSE3QuaternionRightAD>;
    using VariableTupleType    = typename BaseType::VariableTupleType;
    using ADErrorVectorType    = typename BaseType::ADErrorVectorType;
    using MeasurementTypeAD    = Isometry3_<DualValuef>;
    using EstimateType         = VariableSE3QuaternionRightAD::EstimateType;
    using MeasurementOwnerType = MeasurementOwnerEigen_<EstimateType>;

    ADErrorVectorType operator()(VariableTupleType& vars_) final {
      const Isometry3_<DualValuef>& T  = vars_.at<0>()->adEstimate();
      Isometry3_<DualValuef> error_SE3 = _inv_measure * T;
      return geometry3d::t2v(error_SE3);
    }

    void setMeasurement(const MeasurementType& measurement_) override {
      MeasurementOwnerType::setMeasurement(measurement_);
      convertMatrix(_inv_measure, _measurement.inverse());
    }

  protected:
    MeasurementTypeAD _inv_measure =
      MeasurementTypeAD::Identity(); /**< Cache the inverse ad measurement */
  };

} // namespace srrg2_solver
