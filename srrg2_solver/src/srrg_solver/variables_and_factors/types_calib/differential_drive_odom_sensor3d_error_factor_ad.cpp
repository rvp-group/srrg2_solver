#include "differential_drive_odom_sensor3d_error_factor_ad.h"

//! include this: this contains all the implementations of the factors
//! that are hidden to the modules that do not need them to avoid excessive compilation times (EVIL)
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/ad_error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  INSTANTIATE(DifferentialDriveOdomSensor3DErrorFactorAD)

} // namespace srrg2_solver
