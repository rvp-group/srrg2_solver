//! this is the one cpp file where we instantiate all the classes in the library
//! as we will use explicit instantiation in the solvers
//! to prevent monthly compilation times
//! when you add a new type you should visit this file and add the instances
//! of classes you declared in other modules

//! include this: this contains all the implementations of the solver
//! that are hidden to the modules that do not need them to avoid excessive compilation times (EVIL)
#include "srrg_solver/solver_core/instance_macros.h"
#include "srrg_solver/solver_core/solvers_all_impl.cpp"

//! include all the types you declared here
#include "instances.h"

//! we try to instantiate all solvers
namespace srrg2_solver {
  using namespace srrg2_core;

  // this is the function you have to call to initialize
  // the serialization subsystem
  void registerTypes2D() {
    BOSS_REGISTER_CLASS(VariableSE2Right);
    BOSS_REGISTER_CLASS(VariableSE2RightAD);
    BOSS_REGISTER_CLASS(VariablePoint2AD);
    BOSS_REGISTER_CLASS(VariablePoint2);

    BOSS_REGISTER_CLASS(SE2PosePoseGeodesicErrorFactor);
    BOSS_REGISTER_CLASS(SE2PosePoseChordalErrorFactor);

    BOSS_REGISTER_CLASS(SE2PosePointErrorFactor);

    BOSS_REGISTER_CLASS(SE2PosePointBearingErrorFactor);
    BOSS_REGISTER_CLASS(SE2PriorErrorFactor);
    BOSS_REGISTER_CLASS(SE2Point2PointErrorFactorCorrespondenceDriven);
    BOSS_REGISTER_CLASS(SE2Plane2PlaneErrorFactorCorrespondenceDriven);
    BOSS_REGISTER_CLASS(SE2Point2PointWithSensorErrorFactorADCorrespondenceDriven);
    BOSS_REGISTER_CLASS(SE2Plane2PlaneWithSensorErrorFactorADCorrespondenceDriven);
  }
} // namespace srrg2_solver
