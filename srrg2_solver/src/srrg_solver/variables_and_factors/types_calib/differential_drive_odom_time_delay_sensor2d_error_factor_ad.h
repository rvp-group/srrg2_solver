#pragma once
#include "differential_drive_odom_predictor_ad.h"
#include <srrg_geometry/geometry2d.h>
#include <srrg_solver/variables_and_factors/types_2d/variable_se2_ad.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h>
#include <srrg_solver/variables_and_factors/types_calib/variable_time_ad.h>
#include <srrg_solver/solver_core/ad_error_factor.h>

namespace srrg2_solver {

  using TimeIsometry2Map  = std::map<float, Isometry2f>;
  using TimeIsometry2Pair = std::pair<float, Isometry2f>;

  class DifferentialDriveOdomTimeDelaySensor2DErrorFactorAD
    : public ADErrorFactor_<3, VariableTimeAD, VariablePoint3AD, VariableSE2RightAD>,
      public DifferentialDriveOdomPredictorAD {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BaseType = ADErrorFactor_<3, VariableTimeAD, VariablePoint3AD, VariableSE2RightAD>;
    using VariableTupleType = typename BaseType::VariableTupleType;
    using ADErrorVectorType = typename BaseType::ADErrorVectorType;
    using Matrix3adf        = Matrix3_<DualValuef>;
    using Isometry2adf      = Isometry2_<DualValuef>;

    ADErrorVectorType operator()(VariableTupleType& vars) final {
      //! to retrieve a variable you should know the position in the parameter pack
      //! and the type
      const DualValuef& time_delay                = vars.at<0>()->adEstimate()(0);
      const Vector3_<DualValuef>& dd_params       = vars.at<1>()->adEstimate();
      const Isometry2_<DualValuef>& sensor_offset = vars.at<2>()->adEstimate();
      Isometry2_<DualValuef> robot_motion         = Isometry2_<DualValuef>::Identity();

      ADErrorVectorType e;
      e.setZero();
      // bdc here extract the relative measures given period, current time delay and dataset
      Isometry2adf sensor_relative;
      if (!getRelativeMeasure(sensor_relative,
                              _sensor_dataset,
                              _current_time_ad + time_delay,
                              _current_time_ad + _time_period_ad + time_delay)) {
        this->_is_valid = false;
        return e;
      }
      Isometry2adf inverse_measurement_ad = sensor_relative.inverse();

      // do the computation (all in dual value)
      Vector3_<DualValuef> robot_motion_v = _predictRobotMotion(dd_params);

      robot_motion = srrg2_core::geometry2d::v2t(robot_motion_v);

      Isometry2_<DualValuef> sensor_motion_prediction =
        sensor_offset.inverse() * robot_motion * sensor_offset;
      Isometry2_<DualValuef> sensor_motion_error_isometry =
        inverse_measurement_ad * sensor_motion_prediction;

      return geometry2d::t2v<DualValuef>(sensor_motion_error_isometry);
    }

    bool interpolateInData(Isometry2adf& interpolated_ad,
                           TimeIsometry2Map* data_,
                           const DualValuef& time) {
      TimeIsometry2Map::iterator lower, upper;
      interpolated_ad.setIdentity();

      // bdc get lower bound
      lower = data_->lower_bound(time.value);
      if (lower == data_->end() || lower == data_->begin()) {
        return false;
      }
      // bdc get previous element
      lower--;
      upper = data_->upper_bound(time.value);
      if (upper == data_->end() || upper == data_->begin()) {
        return false;
      }

      const DualValuef lower_time_ad(lower->first);
      const DualValuef upper_time_ad(upper->first);

      // bdc compute relative interpolation time
      const DualValuef start_interpolation_time =
        (time - lower_time_ad) / (upper_time_ad - lower_time_ad);

      // bdc interpolate isometries at a certain time
      Matrix3adf m_start, m_end;
      convertMatrix(m_start, lower->second.matrix());
      convertMatrix(m_end, upper->second.matrix());
      // bdc linear interpolation of matrices (in chordal fashion)
      Matrix3adf interpolated = linearInterpolate(start_interpolation_time, m_start, m_end);
      Isometry2adf interpolated_result(interpolated);
      interpolated_ad = interpolated_result;

      return true;
    }

    Matrix3adf linearInterpolate(const DualValuef& start_interpolation_time,
                                 const Matrix3adf& m_start,
                                 const Matrix3adf& m_end) {
      Matrix3adf result;
      result =
        m_start * (DualValuef(1.0) - start_interpolation_time) + start_interpolation_time * m_end;
      return result;
    }

    bool getRelativeMeasure(Isometry2adf& relative_measure_,
                            TimeIsometry2Map* data_,
                            const DualValuef& start,
                            const DualValuef& stop) {
      if (!data_)
        throw std::runtime_error("[SE3PoseTimeErrorFunctorAD::interpolateMeasure]: missing data");

      // bdc find starting isometry
      Isometry2adf start_iso;
      if (!interpolateInData(start_iso, data_, start))
        return false;

      Isometry2adf stop_iso;
      if (!interpolateInData(stop_iso, data_, stop))
        return false;
      relative_measure_ = start_iso.inverse() * stop_iso;
      return true;
    }

    void setSensorDataset(TimeIsometry2Map* sensor_dataset_) {
      _sensor_dataset = sensor_dataset_;
    }

    void setCurrentTime(const float& current_time_) {
      _current_time_ad = DualValuef(current_time_);
    }

    void setTimePeriod(const float& time_period_) {
      _time_period_ad = DualValuef(time_period_);
    }

    void serialize(ObjectData& odata, IdContext& context) override {
      BaseType::serialize(odata, context);
      ArrayData* mdata = new ArrayData;
      for (int i = 0; i < 2; ++i) {
        mdata->add(ticks()[i]);
      }
      odata.setField("ticks", mdata);
    }

    void deserialize(ObjectData& odata, IdContext& context) override {
      BaseType::deserialize(odata, context);
      Vector2f ticks;
      ArrayData* mdata = dynamic_cast<ArrayData*>(odata.getField("ticks"));
      for (int i = 0; i < 2; ++i) {
        ticks[i] = (*mdata)[i].getFloat();
      }
      setTicks(ticks);
    }

  protected:
    DualValuef _current_time_ad;
    DualValuef _time_period_ad;
    TimeIsometry2Map* _sensor_dataset = 0;
  };

} // namespace srrg2_solver
