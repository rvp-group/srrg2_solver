#include "factor_base.h"
#include "robustifier.h"

namespace srrg2_solver {

  bool FactorBase::robustify() {
    _stats.status     = FactorStats::Status::Inlier;
    _stats.kernel_chi = _stats.chi;
    _kernel_scales << _stats.chi, 1., 0;
    if (!_robustifier) {
      return false;
    }
    bool robustified  = _robustifier->robustify(_kernel_scales, _stats.chi);
    _stats.kernel_chi = _kernel_scales[0];
    if (robustified) {
      _stats.status = FactorStats::Status::Kernelized;
    }
    return true;
  }

  void FactorBase::_drawImpl(ViewerCanvasPtr canvas_) const {
    if (!canvas_)
      throw std::runtime_error("FactorBase::draw|invalid canvas");
    std::cerr << "FactorBase::draw not implemented" << std::endl;
  }

} // namespace srrg2_solver
