#include "se2_point2point_left_error_factor_ad.h"
#include <srrg_geometry/geometry3d.h>

//! include this: this contains all the implementations of the factors
//! that are hidden to the modules that do not need them to avoid excessive compilation times (EVIL)
#include <srrg_solver/solver_core/ad_error_factor_impl.cpp>
#include <srrg_solver/solver_core/error_factor_impl.cpp>
#include <srrg_solver/solver_core/instance_macros.h>

namespace srrg2_solver_extras {

  INSTANTIATE(SE2Point2PointLeftErrorFactorAD)
  INSTANTIATE(SE2Point2PointLeftErrorFactorCorrespondenceDrivenAD)
  
} // namespace srrg2_solver
