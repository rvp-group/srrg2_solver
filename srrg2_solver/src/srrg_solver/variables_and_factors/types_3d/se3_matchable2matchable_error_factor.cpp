#include "se3_matchable2matchable_error_factor.h"

//! include this: this contains all the implementations of the factors
//! that are hidden to the modules that do not need them to avoid excessive compilation times (EVIL)
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {

  // ia we do not consider underdetermided constraints (e.g. observing a plane with a point)
  SE3Matchable2MatchableEulerLeftErrorFactor::MatchableTypePairActiveComponentsMap
    SE3Matchable2MatchableEulerLeftErrorFactor::_active_components_factor_map = {
      {std::make_pair(MatchableBase::Type::Point, MatchableBase::Type::Point),
       ActiveComponents{1, 0, 0}},
      //      {std::make_pair(MatchableBase::Type::Point, MatchableBase::Type::Line),
      //       ActiveComponents{1, 0, 0}},
      //      {std::make_pair(MatchableBase::Type::Point, MatchableBase::Type::Plane),
      //       ActiveComponents{1, 0, 0}},
      {std::make_pair(MatchableBase::Type::Line, MatchableBase::Type::Point),
       ActiveComponents{1, 0, 0}},
      {std::make_pair(MatchableBase::Type::Line, MatchableBase::Type::Line),
       ActiveComponents{1, 1, 0}},
      //      {std::make_pair(MatchableBase::Type::Line, MatchableBase::Type::Plane),
      //       ActiveComponents{1, 0, 1}},
      {std::make_pair(MatchableBase::Type::Plane, MatchableBase::Type::Point),
       ActiveComponents{1, 0, 0}},
      {std::make_pair(MatchableBase::Type::Plane, MatchableBase::Type::Line),
       ActiveComponents{1, 0, 1}},
      {std::make_pair(MatchableBase::Type::Plane, MatchableBase::Type::Plane),
       ActiveComponents{1, 1, 0}}};

  SE3Matchable2MatchableEulerLeftErrorFactor::SE3Matchable2MatchableEulerLeftErrorFactor() :
    SE3Matchable2MatchableEulerLeftErrorFactor::BaseType() {
    //ia call only base type contructor
  }

  SE3Matchable2MatchableEulerLeftErrorFactor::~SE3Matchable2MatchableEulerLeftErrorFactor() {
  }

  inline void SE3Matchable2MatchableEulerLeftErrorFactor::errorAndJacobian(bool error_only_) {
    using Matrix1_6f = Eigen::Matrix<float, 1, 6>;

    // ia we first have to understand the type of factor
    std::pair<int, int> matchables_types_pair(_fixed_matchable->type(), _moving_matchable->type());

    const Isometry3f& X = _variables.at<0>()->estimate();
    // ia matchable of the new frame (in sensor coordinates)
    const Matrix3f fixed_rotation_t = _fixed_matchable->rotation().transpose();
    // ia transformed matchable of the new frame (in world coordinates)
    const Matchablef fixed_transformed         = _fixed_matchable->transform(X);
    const Vector3f& fixed_transformed_origin   = fixed_transformed.origin();
    const Vector3f fixed_transformed_direction = fixed_transformed.direction();

    // ia matchable of the map (in world coordinates)
    const Vector3f& moving_origin    = _moving_matchable->origin();
    const Matrix3f& moving_rotation  = _moving_matchable->rotation();
    const Vector3f moving_direction  = _moving_matchable->direction();
    const Matrix3f moving_rotation_t = moving_rotation.transpose();

    const Matrix3f skew_fixed_transformed_origin =
      srrg2_core::geometry3d::skew(fixed_transformed_origin);

    // ia compute error
    _e.setZero();
    _e.head(3) = moving_rotation_t * (fixed_transformed_origin - moving_origin);
    if (_active_components_factor_map.at(matchables_types_pair).direction) {
      _e.segment<3>(3) = fixed_transformed_direction - moving_direction;
    }
    if (_active_components_factor_map.at(matchables_types_pair).orthogonality) {
      _e[6] = fixed_transformed_direction.dot(moving_direction);
    }

    if (error_only_) {
      return;
    }

    // ia jacobians
    Matrix3_6f J_origin    = Matrix3_6f::Zero(); // ia d_e_origin/d_Dx
    Matrix3_6f J_direction = Matrix3_6f::Zero(); // ia d_e_direction/d_Dx
    Matrix1_6f J_ortho     = Matrix1_6f::Zero(); // ia d_e_ortho/d_Dx

    J_origin.block<3, 3>(0, 0) = moving_rotation_t;
    J_origin.block<3, 3>(0, 3) = -moving_rotation_t * skew_fixed_transformed_origin;

    if (_active_components_factor_map.at(matchables_types_pair).direction) {
      J_direction.block<3, 3>(0, 3) = -srrg2_core::geometry3d::skew(fixed_transformed_direction);
    }

    if (_active_components_factor_map.at(matchables_types_pair).orthogonality) {
      const Matrix3f skew_moving_direction = srrg2_core::geometry3d::skew(moving_direction);
      J_ortho.block<1, 3>(0, 3) =
        (fixed_rotation_t * X.linear().transpose() * skew_moving_direction).row(2);
    }

    // ia compose the 7x6 jacobian
    _J.setZero();
    _J.block<3, 6>(0, 0) = J_origin;
    _J.block<3, 6>(3, 0) = J_direction;
    _J.block<1, 6>(6, 0) = J_ortho;
  }

  INSTANTIATE(SE3Matchable2MatchableEulerLeftErrorFactorCorrespondenceDriven)

} // namespace srrg2_solver
