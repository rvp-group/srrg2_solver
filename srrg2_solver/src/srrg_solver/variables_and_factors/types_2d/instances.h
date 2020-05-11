#pragma once

#include "se2_point2point_error_factor.h"
#include "se2_plane2plane_error_factor.h"
#include "se2_prior_error_factor.h"

#include "se2_pose_pose_chordal_error_factor.h"
#include "se2_pose_pose_geodesic_error_factor.h"

#include "se2_pose_point_error_factor.h"

#include "se2_pose_point_bearing_error_factor.h"

#include "variable_point2.h"
#include "variable_point2_ad.h"
#include "variable_se2.h"
#include "variable_se2_ad.h"

namespace srrg2_solver {
  void variables_and_factors_2d_registerTypes() __attribute__((constructor)) ;
} // namespace srrg2_solver
