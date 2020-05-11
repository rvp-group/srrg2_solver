#pragma once
#include "srrg_solver/solver_core/error_factor.h"
#include "srrg_solver/solver_core/factor_correspondence_driven_dynamic.h"
#include "variable_se3.h"
#include <srrg_matchable/matchable.h>

namespace srrg2_solver {

  /**
   * @brief This is a single matchable-matchable factor. Used to register 3D matchable scenes in a
   * ICP fashion. The variable estimated is SE3 Euler-Left. Error is a 7D vector [e_origin
   * e_direction e_orthogonality] computed between predicted and fixed matchable.
   */
  class SE3Matchable2MatchableEulerLeftErrorFactor : public ErrorFactor_<7, VariableSE3EulerLeft> {
  public:
    using VariableType = VariableSE3EulerLeft;
    using FixedType    = Matchablef;
    using MovingType   = Matchablef;
    using BaseType     = ErrorFactor_<7, VariableSE3EulerLeft>;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // ia required to not complain
    SE3Matchable2MatchableEulerLeftErrorFactor();
    virtual ~SE3Matchable2MatchableEulerLeftErrorFactor();

    /**
     * @brief Set a matchable measurement as fixed entry in the ICP-like optimization process
     * @param[in] Matchablef fixed_: the fixed matchable.
     */
    inline void setFixed(const Matchablef& fixed_) {
      _fixed_matchable = &fixed_;
    }

    /**
     * @brief Set a matchable measurement as moving entry in the ICP-like optimization process.
     * The prediction h(x) is computed as: X * moving
     * @param[in] Matchablef moving_: the moving matchable.
     */
    inline void setMoving(const Matchablef& moving_) {
      _moving_matchable = &moving_;
    }

    void errorAndJacobian(bool error_only_) final;

  protected:
    const Matchablef* _fixed_matchable  = nullptr; // ia new matchables
    const Matchablef* _moving_matchable = nullptr; // ia map matchables

  private:
    /**
     * @brief Auxiliary structure that tells which component of the 7D error vector is active (given
     * the types of the fixed and moving matchable)
     */
    struct ActiveComponents {
      bool origin        = false;
      bool direction     = false;
      bool orthogonality = false;
    };

    /**
     * @brief Auxiliary struct that provides hashing for fast reference in the error computation
     */
    struct PairHasher {
      template <class T1, class T2>
      std::size_t operator()(const std::pair<T1, T2>& pair) const {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
      }
    };

    // ia map key is <fixed_type, moving_type>
    using MatchableTypePair = std::pair<int, int>;
    // ia actual map type
    using MatchableTypePairActiveComponentsMap =
      std::unordered_map<MatchableTypePair, ActiveComponents, PairHasher>;

    /**
     * @brief Map that from the association <fixed,moving> returns the active components of the
     * factor
     */
    static MatchableTypePairActiveComponentsMap _active_components_factor_map;
  };

  using SE3Matchable2MatchableEulerLeftErrorFactorCorrespondenceDriven =
    FactorCorrespondenceDrivenDynamic_<SE3Matchable2MatchableEulerLeftErrorFactor>;

} // namespace srrg2_solver
