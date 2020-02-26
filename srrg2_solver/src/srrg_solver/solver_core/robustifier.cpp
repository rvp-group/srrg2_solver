#include "robustifier.h"
#include "factor_base.h"

namespace srrg2_solver {

  bool RobustifierSaturated::robustify(srrg2_core::Vector3f& scales_, const float chi_) const {
    scales_ << chi_, 1, 0;
    const float& t = param_chi_threshold.value();
    if (chi_ > t) {
      scales_[0] = t;
      scales_[1] = 1 / t;
      return true;
    } else {
      return false;
    }
  }

  bool RobustifierCauchy::robustify(srrg2_core::Vector3f& scales_, const float chi_) const {
    const float& t    = param_chi_threshold.value();
    const float inv_t = 1 / t;
    const float aux   = inv_t * chi_ + 1;
    scales_[0]        = t * std::log(aux);
    scales_[1]        = 1 / aux;
    scales_[2]        = -inv_t * std::pow(scales_[1], 2);
    return chi_ > t;
  }

  bool RobustifierClamp::robustify(srrg2_core::Vector3f& scales_, const float chi_) const {
    scales_ << chi_, 1, 0;
    if (chi_ > param_chi_threshold.value()) {
      scales_.setZero();
      return true;
    } else {
      return false;
    }
  }

} // namespace srrg2_solver
