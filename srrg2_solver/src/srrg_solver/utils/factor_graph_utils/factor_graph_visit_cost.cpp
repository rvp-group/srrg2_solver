#include "factor_graph_visit_cost.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  

  float FactorGraphVisitCostUniform::cost(const FactorBase& factor,
                                          int var_from_pos,
                                          int var_to_pos){
    assert(var_from_pos>=0);
    assert(var_to_pos<factor.numVariables());
    return param_factor_cost.value();
  }
}
