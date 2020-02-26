#pragma once
#include "se3_projective_error_factor.h"
#include "se3_projective_depth_error_factor.h"
#include "se3_rectified_stereo_projective_error_factor.h"

namespace srrg2_solver{
  void projective_registerTypes() __attribute__((constructor)) ;
}
