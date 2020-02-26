//! this is the one cpp file where we instantiate all the classes in the library
//! as we will use explicit instantiation in the solvers
//! to prevent monthly compilation times
//! when you add a new type you should visit this file and add the instances
//! of classes you declared in other modules

//! include this: this contains all the implementations of the solver
//! that are hidden to the modules that do not need them to avoid excessive compilation times (EVIL)
#include "srrg_solver/solver_core/solvers_all_impl.cpp"

//! include all the types you declared here
#include "instances.h"

//! we try to instantiate all solvers
namespace srrg2_solver {
  using namespace srrg2_core;

  // this is the function you have to call to initialize
  // the serialization subsystem
  void registerTypes3D() {
    BOSS_REGISTER_CLASS(VariablePoint3AD);
    BOSS_REGISTER_CLASS(VariablePoint3);
    BOSS_REGISTER_CLASS(VariableMatchable);
    BOSS_REGISTER_CLASS(VariableSE3QuaternionRight);
    BOSS_REGISTER_CLASS(VariableSE3QuaternionRightAD);
    BOSS_REGISTER_CLASS(VariableSE3QuaternionLeft);
    BOSS_REGISTER_CLASS(VariableSE3QuaternionLeftAD);
    BOSS_REGISTER_CLASS(VariableSE3EulerRight);
    BOSS_REGISTER_CLASS(VariableSE3EulerRightAD);
    BOSS_REGISTER_CLASS(VariableSE3EulerLeft);
    BOSS_REGISTER_CLASS(VariableSE3EulerLeftAD);
    BOSS_REGISTER_CLASS(SE3PoseMotionErrorFactorDataDriven);
    BOSS_REGISTER_CLASS(SE3RelativeSensorMotionErrorFactorAD);
    // ia pose pose
    BOSS_REGISTER_CLASS(SE3PosePoseChordalEulerLeftErrorFactor);
    BOSS_REGISTER_CLASS(SE3PosePoseGeodesicErrorFactor);

    // ia pose point
    BOSS_REGISTER_CLASS(SE3PosePointOffsetErrorFactor);
    BOSS_REGISTER_CLASS(SE3PosePointErrorFactor);

    // // ia pose matchables
    BOSS_REGISTER_CLASS(SE3PoseMatchableEulerLeftErrorFactor);
    BOSS_REGISTER_CLASS(SE3Matchable2MatchableEulerLeftErrorFactor);
    BOSS_REGISTER_CLASS(SE3Matchable2MatchableEulerLeftErrorFactorCorrespondenceDriven)

    // // many factors
    BOSS_REGISTER_CLASS(SE3Plane2PlaneErrorFactor);
    BOSS_REGISTER_CLASS(SE3Plane2PlaneErrorFactorCorrespondenceDriven);
    BOSS_REGISTER_CLASS(SE3Point2PointErrorFactor);
    BOSS_REGISTER_CLASS(SE3Point2PointErrorFactorCorrespondenceDriven);
    BOSS_REGISTER_CLASS(SE3PriorErrorFactorAD);
    BOSS_REGISTER_CLASS(SE3PriorOffsetErrorFactorAD);
  }
} // namespace srrg2_solver
