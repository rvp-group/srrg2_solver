#pragma once
#include "variable_se3.h"
#include <srrg_solver/solver_core/factor.h>
#include <srrg_solver/solver_core/measurement_owner.h>
#include <srrg_solver/solver_core/variable_ptr_tuple.h>

/* This factor is still under development, at the current state it must be considered as
 * DEPRECATED*/

namespace srrg2_solver {
  using namespace srrg2_core;
  using PosePoseVariableTuple = VariablePtrTuple_<VariableSE3EulerLeft, VariableSE3EulerLeft>;

  class SE3PosePoseChordalHessianFactor : public Factor_<PosePoseVariableTuple>,
                                          public MeasurementOwnerEigen_<Isometry3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType                  = Factor_<PosePoseVariableTuple>;
    using ThisType                  = SE3PosePoseChordalHessianFactor;
    using MeasurementOwnerType      = MeasurementOwnerEigen_<Isometry3f>;
    using PoseInformationMatrixType = Matrix6f;
    using ErrorVectorType           = Eigen::Matrix<float, 12, 1>;
    using InformationMatrixType     = Eigen::Matrix<float, 12, 12>;
    using JacobianMatrixType        = Eigen::Matrix<float, 12, 6>;
    SE3PosePoseChordalHessianFactor();

    void setInformationMatrix(const PoseInformationMatrixType& omega_);

    inline void setInformationMatrix(const InformationMatrixType& omega_) {
      _omega = omega_;
    }

    inline InformationMatrixType informationMatrix() const {
      return _omega;
    }

    inline PoseInformationMatrixType poseInformationMatrix() const {
      return _pose_omega;
    }

    void setMeasurement(const Isometry3f& measurement_) final;

    int measurementDim() const override {
      return 6;
    }

    bool isValid() const override {
      return true;
    }

    void compute(bool chi_only = false, bool force = false) final;

    /*! Serialize the measurement contained in the factor */
    void serialize(ObjectData& odata, IdContext& context) final;
    /*! Deserialize the measurement contained in the factor */
    void deserialize(ObjectData& odata, IdContext& context) final;

  protected:
    Matrix3f Rx0, Ry0, Rz0;
    InformationMatrixType _omega;
    PoseInformationMatrixType _pose_omega;
  };
}
