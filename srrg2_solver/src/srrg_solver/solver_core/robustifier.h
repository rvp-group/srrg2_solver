#pragma once
#include "solver_stats.h"
#include <srrg_config/property_configurable.h>
#include <srrg_geometry/geometry_defs.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  /*! @brief Basic interface for a robustifier (M-estimator), you need to overrided the robustify()
    method to define your own robustifier */
  class RobustifierBase : public Configurable {
  public:
    PARAM(PropertyFloat, chi_threshold, "threshold of chi after which the kernel is active", 1, 0);
    /*! Apply the robust kernel to the cost function
      @param[out] scales_ scales_[0]: robustified error, scales_[1]: 1st derivative of kernel,
      scales_[2]: 2nd derivative of kernel
      @param[in] chi_ current value of the squared error
      @return true if kernel was active
    */
    virtual bool robustify(srrg2_core::Vector3f& scales_, const float chi_) const = 0;
  };

  using RobustifierBasePtr =
    std::shared_ptr<RobustifierBase>; /*!<Shared pointer to RobustifierBase */

  /*! @brief Satured robust kernel, if the cost function is above chi_threshold the robustified
   error contributes with a constant value*/
  struct RobustifierSaturated : public RobustifierBase {
    bool robustify(srrg2_core::Vector3f& scales_, const float chi_) const override;
  };

  /*! @brief Cauchy robust kernel, map the squared error to a logarithmic scale*/
  struct RobustifierCauchy : public RobustifierBase {
    bool robustify(srrg2_core::Vector3f& scales_, const float chi_) const override;
  };

  //! @brief Clamp robust kernel, outliers are clamped completely (the corresponding cost is 0) from
  //! the optimization: enables inlier only runs */
  struct RobustifierClamp : public RobustifierBase {
    bool robustify(srrg2_core::Vector3f& scales_, const float chi_) const override;
  };

} // namespace srrg2_solver
