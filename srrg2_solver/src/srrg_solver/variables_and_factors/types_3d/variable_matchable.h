#pragma once
#include <srrg_matchable/matchable.h>

#include "srrg_solver/solver_core/variable.h"

namespace srrg2_solver {
  class VariableMatchable : public VariableGeneric_<5, srrg2_core::Matchable_> {
  public:
    using BaseVariableType = Variable_<5, srrg2_core::Matchable_>;

    //! @brief object life (ds style)
    ~VariableMatchable();

    void setZero();

    void applyPerturbation(const srrg2_core::Vector5f& pert_) override;

    //! @brief serialization of the variable through BOSS.
    void serialize(ObjectData& odata, IdContext& context) override;
    
    //! @brief deserialization of the variable through BOSS.
    void deserialize(ObjectData& odata, IdContext& context) override;
    
    void _drawImpl(ViewerCanvasPtr canvas_) const override;
  };
} // namespace srrg2_solver
