#pragma once
#include "robustifier.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  class FactorBase;
  /*! @brief Base interface for a robustifier policy, which assign a robust kernel to a factor based
    on some rule. To define your own policy you need to override getRobustifier() and specify inside
    it the matching rule */
  class RobustifierPolicyBase : public Configurable {
  public:
    PARAM(PropertyConfigurable_<RobustifierBase>,
          robustifier,
          "robustifier matching the rule",
          0,
          0);

    virtual ~RobustifierPolicyBase() {
    }
    /*! Get a robustifier if factor match the rule
      @param[in] factor
      @return Corresponding robustifier
    */
    virtual RobustifierBasePtr getRobustifier(FactorBase* factor) = 0;
  };

  using RobustifierPolicyBasePtr =
    std::shared_ptr<RobustifierPolicyBase>; /*!<Shared pointer to
                                              RobustifierPolicyBase */
  /*! @brief The robustifier is assigned to the factor if the class name of the factor matches
    config->className()*/
  class RobustifierPolicyByType : public RobustifierPolicyBase {
  public:
    PARAM(Property_<std::string>,
          factor_class_name,
          "name of the class of the matching factor",
          "",
          0);

    virtual ~RobustifierPolicyByType() {
    }

    RobustifierBasePtr getRobustifier(FactorBase* factor) override;
  };

  using RobustifierPolicyByTypePtr =
    std::shared_ptr<RobustifierPolicyByType>; /*!< Shared pointer
                                                to RobustifierPolicyByType */

} // namespace srrg2_solver
