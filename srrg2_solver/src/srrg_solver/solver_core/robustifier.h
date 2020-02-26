#pragma once
#include "solver_stats.h"
#include <srrg_config/property_configurable.h>
#include <srrg_geometry/geometry_defs.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  class MultiFactorBase;

  class RobustifierBase : public Configurable {
  public:
    PARAM(PropertyFloat, chi_threshold, "threshold of chi after which the kernel is active", 1, 0);

    // scales[0]: robustified error
    // scales[1]: 1st derivative of kernel
    // scales[2]: 2nd derivative of kernel
    // returns true if kernel was active
    virtual bool robustify(srrg2_core::Vector3f& scales_, const float chi_) const = 0;
  };

  using RobustifierBasePtr = std::shared_ptr<RobustifierBase>;

  //! outlier error is capped at param_chi_threshold: pulling effect
  struct RobustifierSaturated : public RobustifierBase {
    bool robustify(srrg2_core::Vector3f& scales_, const float chi_) const override;
  };

  //! outlier error contributes with logarithmicly increasing pull
  struct RobustifierCauchy : public RobustifierBase {
    bool robustify(srrg2_core::Vector3f& scales_, const float chi_) const override;
  };

  //! outliers are clamped completely from the optimization: enables inlier only runs
  struct RobustifierClamp : public RobustifierBase {
    bool robustify(srrg2_core::Vector3f& scales_, const float chi_) const override;
  };

} // namespace srrg2_solver
