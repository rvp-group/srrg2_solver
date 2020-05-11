#pragma once
#include "srrg_solver/solver_core/ad_error_factor.h"
#include "variable_time_ad.h"
#include <srrg_geometry/ad_quaternion.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  using TimeIsometryMap  = std::map<float, Isometry3f>;
  using TimeIsometryPair = std::pair<float, Isometry3f>;

  /**
   * @brief 3D exstrinsic parameters and time delay error factor.
   * Compute the 3D pose of a sensor mounted on it and the time delay given encoder's readings
   * and the kinematic model of the robot. The error is computed from the box-minus between the
   * measured sensor motion and the predicted one using interpolation
   */
  class SE3SensorPoseTimeDelayErrorFactorAD
    : public ADErrorFactor_<6, VariableTimeAD, VariableSE3QuaternionRightAD> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType          = ADErrorFactor_<6, VariableTimeAD, VariableSE3QuaternionRightAD>;
    using VariableTupleType = typename BaseType::VariableTupleType;
    using ADErrorVectorType = typename BaseType::ADErrorVectorType;
    using Matrix4adf        = Matrix4_<DualValuef>;
    using Isometry3adf      = Isometry3_<DualValuef>;

    ADErrorVectorType operator()(VariableTupleType& vars) final {
      this->_is_valid                = true;
      const DualValuef& time_delay   = vars.at<0>()->adEstimate()(0);
      const Isometry3adf& extrinsics = vars.at<1>()->adEstimate();

      ADErrorVectorType e;
      e.setZero();

      // bdc interpolate using the delay
      // TODO this interpolation is useless since it does not depend on the time delay.
      //      only the sensor measures should be interpolated, since the odometry measures are
      //      supposed to be shared between several sensors. Thus, here relative values can be
      //      directly provided (as in classical problems)
      Isometry3adf odom_relative, sensor_relative;
      if (!getRelativeMeasure(
            odom_relative, _odom_dataset, _current_time_ad, _current_time_ad + _time_period_ad)) {
        this->_is_valid = false;
        return e;
      }
      if (!getRelativeMeasure(sensor_relative,
                              _sensor_dataset,
                              _current_time_ad + time_delay,
                              _current_time_ad + _time_period_ad + time_delay)) {
        this->_is_valid = false;
        return e;
      }
      // bdc compute prediction
      Isometry3adf prediction_ad = extrinsics.inverse() * odom_relative * extrinsics;
      // bdc compute error
      e = geometry3d::t2v(sensor_relative.inverse() * prediction_ad);
      return e;
    }

    /**
     * @brief interpolate dataset at given time
     * @param[out] Isometry3adf interpolated_ad: interpolated transformation
     * @param[in] TimeIsometry2Map* data: pointer to dataset
     * @param[in] DualValuef time: interpolation time
     */
    bool interpolateInData(Isometry3adf& interpolated_ad,
                           TimeIsometryMap* data_,
                           const DualValuef& time) {
      TimeIsometryMap::iterator lower, upper;
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
      Matrix4adf m_start, m_end;
      convertMatrix(m_start, lower->second.matrix());
      convertMatrix(m_end, upper->second.matrix());
      // bdc linear interpolation of matrices (in chordal fashion)
      Matrix4adf interpolated = linearInterpolate(start_interpolation_time, m_start, m_end);
      Isometry3adf interpolated_result(interpolated);
      interpolated_ad = interpolated_result;

      // bdc isometry interpolation using slerp on Quaternion
      //      Isometry3adf m_start_iso(m_start), m_end_iso(m_end);
      //      Isometry3adf m_start_iso(lower->second), m_end_iso(upper->second);
      //      interpolated_ad =  geometry3d::interpolateIsometries(start_interpolation_time,
      //      m_start_iso, m_end_iso);
      return true;
    }

    /**
     * @brief perform linear interpolation between two poses at a given time
     * @param[in] DualValuef start_interpolation_time: interpolation time
     * @param[in] Matrix4adf m_start: starting pose
     * @param[in] Matrix4adf m_end: ending pose
     * @return Matrix4adf interpolated transform
     */
    Matrix4adf linearInterpolate(const DualValuef& start_interpolation_time,
                                 const Matrix4adf& m_start,
                                 const Matrix4adf& m_end) {
      Matrix4adf result;
      result =
        m_start * (DualValuef(1.0) - start_interpolation_time) + start_interpolation_time * m_end;
      return result;
    }

    /**
     * @brief get relative measure
     * @param[out] Isometry3adf relative_measure_: relative measure
     * @param[in] TimeIsometryMap data: dataset
     * @param[in] DualValuef start: starting time
     * @param[in] DualValuef stop: ending time
     * @return bool: returns false if anything went wrong
     */
    bool getRelativeMeasure(Isometry3adf& relative_measure_,
                            TimeIsometryMap* data_,
                            const DualValuef& start,
                            const DualValuef& stop) {
      if (!data_)
        throw std::runtime_error("[SE3PoseTimeErrorFunctorAD::interpolateMeasure]: missing data");

      // bdc find starting isometry
      Isometry3adf start_iso;
      if (!interpolateInData(start_iso, data_, start))
        return false;

      Isometry3adf stop_iso;
      if (!interpolateInData(stop_iso, data_, stop))
        return false;
      relative_measure_ = start_iso.inverse() * stop_iso;
      return true;
    }

    void setMeasurement(const Matrix0f& null_) {
      throw std::runtime_error(
        "[SE3PoseTimeErrorFunctorAD::setMeasurement]: YOU DON'T HAVE TO USE THIS FUNCTION");
    }

    const Matrix0f& measurement() const {
      throw std::runtime_error(
        "[SE3PoseTimeErrorFunctorAD::measurement]: YOU DON'T HAVE TO USE THIS FUNCTION");
    }

    /**
     * @brief set a timestamped odometry map, ordered by timestamp
     * @param[in] TimeIsometryMap odom_dataset_: relative robot motion dataset
     */

    void setOdomDataset(TimeIsometryMap* odom_dataset_) {
      _odom_dataset = odom_dataset_;
    }

    /**
     * @brief set a timestamped sensor_pose map, ordered by timestamp
     * @param[in] TimeIsometryMap sensor_dataset: sensor motion dataset
     */
    void setSensorDataset(TimeIsometryMap* sensor_dataset_) {
      _sensor_dataset = sensor_dataset_;
    }

    /**
     * @brief set current time
     * @param[in] float current_time: time at which to perform interpolation
     */
    void setCurrentTime(const float& current_time_) {
      _current_time_ad = DualValuef(current_time_);
    }

    void setTimePeriod(const float& time_period_) {
      _time_period_ad = DualValuef(time_period_);
    }

  protected:
    DualValuef _current_time_ad;          /**< current autodiff time*/
    DualValuef _time_period_ad;           /**< time period autodiff*/
    TimeIsometryMap* _odom_dataset   = 0; /**< pointer to the robot motion dataset*/
    TimeIsometryMap* _sensor_dataset = 0; /**< pointer to the sensor motion dataset*/
  };

} // namespace srrg2_solver
