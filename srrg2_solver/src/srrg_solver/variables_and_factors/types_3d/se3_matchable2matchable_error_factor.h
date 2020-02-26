#pragma once
#include "srrg_solver/solver_core/error_factor.h"
#include "srrg_solver/solver_core/factor_correspondence_driven_dynamic.h"
#include "variable_se3.h"
#include <srrg_matchable/matchable.h>

namespace srrg2_solver {
  class SE3Matchable2MatchableEulerLeftErrorFactor : public ErrorFactor_<7, VariableSE3EulerLeft> {
  public:
    using VariableType = VariableSE3EulerLeft;
    using FixedType    = Matchablef;
    using MovingType   = Matchablef;
    using BaseType     = ErrorFactor_<7, VariableSE3EulerLeft>;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //ia required to not complain
    SE3Matchable2MatchableEulerLeftErrorFactor();
    virtual ~SE3Matchable2MatchableEulerLeftErrorFactor();

    inline void setFixed(const Matchablef& fixed_) {
      _fixed_matchable = &fixed_;
    }

    inline void setMoving(const Matchablef& moving_) {
      _moving_matchable = &moving_;
    }

    void errorAndJacobian(bool error_only_) final;

  protected:
    const Matchablef* _fixed_matchable  = nullptr; // ia new matchables
    const Matchablef* _moving_matchable = nullptr; // ia map matchables

  private:
    // ia map that from the association <fixed,moving> returns the active components of the factor
    struct ActiveComponents {
      bool origin        = false;
      bool direction     = false;
      bool orthogonality = false;
    };

    struct PairHasher {
      template <class T1, class T2>
      std::size_t operator()(const std::pair<T1, T2>& pair) const {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
      }
    };

    // ia map is <fixed_type, moving_type>
    using MatchableTypePair = std::pair<int, int>;
    //    using MatchableTypePairActiveComponentsMap = std::map<MatchableTypePair,
    //    ActiveComponents>;
    using MatchableTypePairActiveComponentsMap =
      std::unordered_map<MatchableTypePair, ActiveComponents, PairHasher>;
    static MatchableTypePairActiveComponentsMap _active_components_factor_map;
  };

  using SE3Matchable2MatchableEulerLeftErrorFactorCorrespondenceDriven =
    FactorCorrespondenceDrivenDynamic_<SE3Matchable2MatchableEulerLeftErrorFactor>;

} // namespace srrg2_solver
