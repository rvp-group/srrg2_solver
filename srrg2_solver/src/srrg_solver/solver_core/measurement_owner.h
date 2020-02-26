#pragma once
#include <Eigen/Core>
#include <srrg_boss/object_data.h>

namespace srrg2_solver {

  using namespace srrg2_core;

  class MeasurementOwnerBase {
  public:
    virtual void serializeMeasurement(ObjectData& odata, IdContext& context)   = 0;
    virtual void deserializeMeasurement(ObjectData& odata, IdContext& context) = 0;
    virtual ~MeasurementOwnerBase() {
    }
  };

  template <typename MeasurementType_>
  class MeasurementOwner_ : public MeasurementOwnerBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using MeasurementType = MeasurementType_;
    virtual void setMeasurement(const MeasurementType& measurement_) {
      _measurement = measurement_;
    }
    virtual const MeasurementType& measurement() const {
      return _measurement;
    }
    virtual ~MeasurementOwner_() {
    }

  protected:
    MeasurementType _measurement;
  };

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
