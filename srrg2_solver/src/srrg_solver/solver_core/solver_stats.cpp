#include "solver_stats.h"

namespace srrg2_solver {

  std::ostream& operator<<(std::ostream& os, const IterationStats& istat) {
    os << "it= " << istat.iteration << "; level= " << istat.level
       << "; num_inliers= " << istat.num_inliers << "; chi_inliers= " << istat.chi_inliers
       << "; num_outliers= " << istat.num_outliers << "; chi_outliers= " << istat.chi_outliers
       << "; chi_normalized= " << istat.chi_normalized
       << "; num_suppressed= " << istat.num_suppressed
       << "; t_iteration= " << istat.t_solve + istat.t_linearize + istat.t_update
       << "; t_linearize= " << istat.t_linearize << "; t_solve= " << istat.t_solve
       << "; t_update= " << istat.t_update << "; lambda=" << istat.lambda
       << "; internal_iters= " << istat.num_internal_iteration;
    return os;
  }

  // debug
  std::ostream& operator<<(std::ostream& os, const IterationStatsVector& stats) {
    using namespace std;
    os << "STATS: begin" << endl;
    for (size_t i = 0; i < stats.size(); ++i) {
      const IterationStats& istat = stats[i];
      os << istat << endl;
    }
    os << "STATS: end" << endl;
    return os;
  }

} // namespace srrg2_solver
