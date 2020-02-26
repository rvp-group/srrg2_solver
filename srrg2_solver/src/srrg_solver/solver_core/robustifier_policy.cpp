#include "robustifier_policy.h"
#include "factor_base.h"

namespace srrg2_solver {

  RobustifierBasePtr RobustifierPolicyByType::getRobustifier(FactorBase* factor) {
    if (param_factor_class_name.value() == factor->className()) {
      return param_robustifier.value();
    }
    return 0;
  }

} // namespace srrg2_solver
