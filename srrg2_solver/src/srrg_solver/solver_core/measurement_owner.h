#pragma once
#include <Eigen/Core>
#include <srrg_boss/object_data.h>

namespace srrg2_solver {

  using namespace srrg2_core;
  /*! @brief Base interface for measurement owner, you need to override
   * serializeMeasurement/deserializeMeasurement*/
  class MeasurementOwnerBase {
  public:
    /*! Serialize the measurement through BOSS - see srrg2_core */
    virtual void serializeMeasurement(ObjectData& odata, IdContext& context) = 0;
    /*! Deserialize the measurement through BOSS - see srrg2_core */
    virtual void deserializeMeasurement(ObjectData& odata, IdContext& context) = 0;
    virtual ~MeasurementOwnerBase() {
    }
  };
  /*! @brief Intermediate class for non-Eigen measurement type, you must specify the template and
    override serializeMeasurement/deserializeMeasurement */
  template <typename MeasurementType_>
  class MeasurementOwner_ : public MeasurementOwnerBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using MeasurementType = MeasurementType_;
    /*! Set the measurement
      @param[in] measurement_
    */
    virtual void setMeasurement(const MeasurementType& measurement_) {
      _measurement = measurement_;
    }
    /*! @return The measurement stored */
    virtual const MeasurementType& measurement() const {
      return _measurement;
    }
    virtual ~MeasurementOwner_() {
    }

  protected:
    MeasurementType _measurement;
  };
  /*! @brief Measurement owner for Eigen type measurement */
  template <typename MeasurementType_>
  class MeasurementOwnerEigen_ : public MeasurementOwner_<MeasurementType_> {
  public:
    using MeasurementType = MeasurementType_;
    void serializeMeasurement(ObjectData& odata, IdContext& context) override;
    void deserializeMeasurement(ObjectData& odata, IdContext& context) override;
    virtual ~MeasurementOwnerEigen_() {
    }
  };

} // namespace srrg2_solver
