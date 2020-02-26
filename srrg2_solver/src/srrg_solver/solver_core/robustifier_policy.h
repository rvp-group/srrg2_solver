#pragma once
#include "robustifier.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  class FactorBase;

  class RobustifierPolicyBase : public Configurable {
  public:
    PARAM(PropertyConfigurable_<RobustifierBase>,
          robustifier,
          "robustifier matching the rule",
          0,
          0);

    virtual ~RobustifierPolicyBase() {
    }

    virtual RobustifierBasePtr getRobustifier(FactorBase* factor) = 0;
  };

  using RobustifierPolicyBasePtr = std::shared_ptr<RobustifierPolicyBase>;

  class RobustifierPolicyByType : public RobustifierPolicyBase {
  public:
    PARAM(Property_<std::string>,
          factor_class_name,
          "name of the class of the matching factor",
          "",
          0);

    virtual ~RobustifierPolicyByType() {
    }

    // returns the robustifier if the class name of the factor matches config->className()
    RobustifierBasePtr getRobustifier(FactorBase* factor) override;
  };

  using RobustifierPolicyByTypePtr = std::shared_ptr<RobustifierPolicyByType>;

} // namespace srrg2_solver
