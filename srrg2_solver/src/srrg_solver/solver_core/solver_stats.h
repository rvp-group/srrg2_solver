#pragma once
#include <Eigen/StdVector>
#include <iostream>
#include <vector>

namespace srrg2_solver {

  struct FactorStats {
    enum Status { Error = 0x0, Inlier = 0x1, Kernelized = 0x2, Suppressed = 0x3, Disabled = 0x4 };
    Status status    = Error;
    float chi        = 0;
    float kernel_chi = 0;
    bool is_updated  = false;
  };

  struct IterationStats {
    enum Type { Full = 0x0, LinearOnly = 0x1 };

    inline void reset() {
      type           = Full;
      iteration      = 0;
      chi_inliers    = 0;
      chi_outliers   = 0;
      chi_normalized = 0;
      num_inliers    = 0;
      num_outliers   = 0;
      num_suppressed = 0;

      t_extra     = 0;
      t_linearize = 0;
      t_solve     = 0;
      t_update    = 0;

      lambda                = 0.f;
      num_internal_iteration = 0;
    }

    Type type            = Full;
    int iteration        = 0;
    int level            = 0;
    float chi_inliers    = 0.f;
    float chi_outliers   = 0.f;
    float chi_normalized = 0.f; // cumulative chi: inliers+kernelized errors
    int num_inliers      = 0;
    int num_outliers     = 0;
    int num_suppressed   = 0; // ds also contains number of invalid points (_is_valid)
    int num_disabled     = 0;
    double t_extra       = 0;
    double t_linearize   = 0;
    double t_solve       = 0;
    double t_update      = 0;
    double lambda        = 0.f;
    int num_internal_iteration =
      0; // ia number of internal iterations of the algorithm (e.g. lm iters)
  };

  using FactorStatsVector = std::vector<FactorStats>;

  using IterationStatsVector = std::vector<IterationStats>;

  // debug
  std::ostream& operator<<(std::ostream& os, const IterationStats& istat);

  std::ostream& operator<<(std::ostream& os, const IterationStatsVector& stats);
} // namespace srrg2_solver
