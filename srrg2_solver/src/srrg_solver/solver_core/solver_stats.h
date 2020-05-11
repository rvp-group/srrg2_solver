#pragma once
#include <Eigen/StdVector>
#include <iostream>
#include <vector>

namespace srrg2_solver {
  /*! @brief Factor statistics, contain a status (inlier/outlier/robust kernel was applied/factor
    was not considered in the optimization) the real squared error and the kernelized one */
  struct FactorStats {
    enum Status { Error = 0x0, Inlier = 0x1, Kernelized = 0x2, Suppressed = 0x3, Disabled = 0x4 };
    Status status    = Error;
    float chi        = 0; /*!< Squared error */
    float kernel_chi = 0; /*!< Squared error with the kernel applied */
    bool is_updated  = false;
  };
  /*! @brief Iteration statistics */
  struct IterationStats {
    enum Type { Full = 0x0, LinearOnly = 0x1 };
    /*! Clear the statistics */
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

      lambda                 = 0.f;
      num_internal_iteration = 0;
    }

    Type type                  = Full; /*!< Type of the statistics, full or just linear system */
    int iteration              = 0;    /*!< Iteration number */
    int level                  = 0; /*!< Level of the optimization, problem might be hierarchical */
    float chi_inliers          = 0.f; /*!< Total squared error for inliers */
    float chi_outliers         = 0.f; /*!< Total squared error for outliers */
    float chi_normalized       = 0.f; /*!< inliers+kernelized squared error */
    int num_inliers            = 0;
    int num_outliers           = 0;
    int num_suppressed         = 0;   /*!< Contains number of invalid points and outliers */
    int num_disabled           = 0;   /*!< Number of factors disabled */
    double t_extra             = 0;   /*!< Timing of extra operations */
    double t_linearize         = 0;   /*!< Time required to linearize the system */
    double t_solve             = 0;   /*!< Time to solver the linear system */
    double t_update            = 0;   /*!< Time to update the solution */
    double lambda              = 0.f; /*!< Value of the damping factor (gn/lm) */
    int num_internal_iteration = 0;   /*!< Number of internal
                                   iterations of the algorithm (e.g. lm iters) */
  };

  using FactorStatsVector = std::vector<FactorStats>;

  using IterationStatsVector = std::vector<IterationStats>;

  /*!< Writer statistics on stream */
  std::ostream& operator<<(std::ostream& os, const IterationStats& istat);

  /*!< Writer statistics on stream */
  std::ostream& operator<<(std::ostream& os, const IterationStatsVector& stats);
} // namespace srrg2_solver
