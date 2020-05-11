#include "solver.h"
#include "srrg_solver/solver_core/internals/sparse_block_matrix/matrix_block_factory.h"
#include <srrg_system_utils/system_utils.h>
#include <sstream>

namespace srrg2_core {
  class ConfigurableShell;
}

namespace srrg2_solver {
  using namespace srrg2_core;
  using namespace std;

  struct CommandLoadGraph : public Configurable::CommandBase {
  public:
    CommandLoadGraph(Solver* solver) :
      Configurable::CommandBase(solver, "load", "load <filename>: loads the graph from filename") {
    }

    bool execute(srrg2_core::ConfigurableShell* shell_,
                 std::string& response,
                 const std::vector<std::string>& tokens) override {
      if (tokens.size() < 2) {
        response = "Solver| load requires a filename";
        return false;
      }
      Solver* solver = dynamic_cast<Solver*>(_configurable);
      if (!solver) {
        throw std::runtime_error("type mismatch");
      }
      std::ostringstream os;
      os << "module: " << _configurable->className() << " ptr: " << _configurable << std::endl;
      os << "reading graph from file [" << tokens[1] << std::endl;
      solver->setGraph(FactorGraph::read(tokens[1]));
      response = os.str();
      return true;
    }
  };

  struct CommandSaveGraph : public Configurable::CommandBase {
  public:
    CommandSaveGraph(Solver* solver) :
      Configurable::CommandBase(solver, "save", "save <filename>: saves the graph to filename") {
    }

    bool execute(srrg2_core::ConfigurableShell* shell_,
                 std::string& response,
                 const std::vector<std::string>& tokens) override {
      if (tokens.size() < 2) {
        response = "Solver| save requires a filename";
        return false;
      }
      Solver* solver = dynamic_cast<Solver*>(_configurable);
      if (!solver) {
        throw std::runtime_error("type mismatch");
      }
      FactorGraphPtr graph = std::dynamic_pointer_cast<FactorGraph>(solver->graph());
      if (!graph) {
        response = "no graph, cannot save";
      }
      std::ostringstream os;
      os << "module: " << _configurable->className() << " ptr: " << _configurable << std::endl;
      os << "saving graph to file [" << tokens[1] << std::endl;
      graph->write(tokens[1]);
      response = os.str();
      return true;
    }
  };

  Solver::Solver() {
    addCommand(new CommandLoadGraph(this));
    addCommand(new CommandSaveGraph(this));
    MatrixBlockFactory* factory = MatrixBlockFactory::instance();
    factory->addAllocator<1, 1>();
    factory->addAllocator<6, 6>();
    factory->addAllocator<6, 3>();
    factory->addAllocator<6, 1>();
    factory->addAllocator<3, 6>();
    factory->addAllocator<3, 3>();
    factory->addAllocator<3, 2>();
    factory->addAllocator<3, 1>();
    factory->addAllocator<2, 3>();
    factory->addAllocator<2, 2>();
    factory->addAllocator<2, 1>();
    factory->addAllocator<1, 2>();
    factory->addAllocator<1, 3>();
    factory->addAllocator<1, 6>();
    // ia allocator for the matchable Jj
    factory->addAllocator<5, 5>();
    factory->addAllocator<7, 5>();
    factory->addAllocator<5, 7>();
    factory->addAllocator<5, 1>();
    factory->addAllocator<1, 5>();
    // ia allocator for the matchable Ji
    factory->addAllocator<5, 6>();
    factory->addAllocator<6, 5>();

    // ia allocator for the chordal
    factory->addAllocator<12, 12>();
    factory->addAllocator<12, 1>();
    factory->addAllocator<1, 12>();

    // ldg allocator for similiarities
    factory->addAllocator<7, 7>();
    factory->addAllocator<7, 1>();
    factory->addAllocator<1, 7>();
  }

  void Solver::allocateStructures() {
    if (!param_linear_solver.value())
      throw std::runtime_error("Solver::allocateStructures|ERROR, no linear solver set");
    param_linear_solver.value()->bindLinearSystem(&_H, &_b);
    SolverBase::allocateStructures();
  }

  void Solver::computeActiveFactors() {
    // we determine the active factors:
    // a factor is active if
    // at least one variable in it is active and no variables are non_active
    // in doing so we reset the indices of all variables encountered
    _active_factors.clear();
    std::map<int, int> num_active_factors_per_level;
    IdFactorPtrContainer& facts = _graph->factors();
    for (auto it = facts.begin(); it != facts.end(); ++it) {
      FactorBase* f = const_cast<FactorBase*>(it.value());
      assert(f && "factor null");
      if (!f->enabled()) {
        continue;
      }
      bool is_active = true;
      bool all_fixed = true;
      for (int pos = 0; pos < f->numVariables(); ++pos) {
        VariableBase* const v = f->variable(pos);
        if (!v) {
          std::cerr << "Solver::computeActiveRegion| unable to cast variable for factor #"
                    << f->graphId() << ". Check VariableType assigned to factor" << std::endl;
          throw std::runtime_error("cast failed");
        }
        assert(v && "variable null at factor");
        v->_hessian_index = -1;
        switch (v->status()) {
          case VariableBase::NonActive:
            is_active = false;
            break;
          case VariableBase::Active:
            all_fixed = false;
            break;
          default:;
        }
      }
      if (is_active && !all_fixed) {
        size_t level = f->level();
        _active_factors[level].push_back(f);
        num_active_factors_per_level.insert(std::make_pair(level, 0));
        num_active_factors_per_level[level] += f->size();
      }
    }
    _factor_stats.clear();
    for (const auto& level_num_factors : num_active_factors_per_level) {
      FactorStatsVector stats;
      stats.resize(level_num_factors.second);
      _factor_stats[level_num_factors.first] = stats;
    }
  }

  void Solver::computeActiveVariables() {
    // we scan the active factors and populate the
    // active variables (non fixed)
    _active_variables.clear();
    // auxiliary set to garanty that the same variable doesnt appear
    // multiple times in the same level
    std::set<VariableBase::Id> variables_attendance_register;
    const std::vector<FactorBase*>& active_factors_level = _active_factors[this->currentLevel()];
    for (auto it = active_factors_level.begin(); it != active_factors_level.end(); ++it) {
      FactorBase* f = *it;
      assert(f && "Solver::computeActiveRegion|active factor null");
      for (int pos = 0; pos < f->numVariables(); ++pos) {
        VariableBase* v = f->variable(pos);
        assert(v && "Solver::computeActiveRegion|variable at active factor null");
        if (v->status() == VariableBase::Active &&
            !variables_attendance_register.count(v->graphId())) {
          v->_hessian_index = _active_variables.size();
          _active_variables.push_back(v);
          variables_attendance_register.insert(v->graphId());
        } else if (v->status() != VariableBase::Active) {
          v->_hessian_index = -1;
        }
      }
    }
  }

  void Solver::computeOrdering() {
    std::vector<IntPair> block_layout;
    auto it = _active_factors.find(this->currentLevel());
    if (it == _active_factors.end()) {
      return;
    }
    std::vector<FactorBase*>& active_factors_level = it->second;
    for (auto it = active_factors_level.begin(); it != active_factors_level.end(); ++it) {
      FactorBase* factor = *it;
      assert(factor && "Solver::computeOrdering|factor null");
      int nvars = factor->numVariables();
      for (int r = 0; r < nvars; ++r) {
        VariableBase* var_r = factor->variable(r);
        assert(var_r && "Solver::computeOrdering|var_r null");
        if (var_r->_hessian_index < 0) {
          continue;
        }
        int idx_r = var_r->_hessian_index;
        for (int c = r; c < nvars; ++c) {
          VariableBase* var_c = factor->variable(c);
          assert(var_c && "Solver::computeOrdering|var_c null");
          if (var_c->_hessian_index < 0) {
            continue;
          }
          int idx_c = var_c->_hessian_index;

          if (idx_r < idx_c) {
            std::swap(idx_r, idx_c);
          }
          block_layout.push_back(std::make_pair(idx_r, idx_c));
        }
      }
    }
    // sort, column major
    std::sort(
      block_layout.begin(), block_layout.end(), [](const IntPair& a, const IntPair& b) -> bool {
        return a.second < b.second || (a.second == b.second && a.first < b.first);
      });
    // remove duplicates
    std::vector<IntPair>::iterator last = std::unique(block_layout.begin(), block_layout.end());
    block_layout.erase(last, block_layout.end());
    // compute ordering
    std::vector<int> ordering;
    ordering.resize(_active_variables.size());
    SparseBlockLinearSolverPtr _linear_solver = param_linear_solver.value();
    _linear_solver->computeOrderingHint(ordering, block_layout);
    std::vector<VariableBase*> src(_active_variables);
    for (size_t i = 0; i < _active_variables.size(); ++i) {
      _active_variables[i] = src[ordering[i]];
    }
  }

  void Solver::printAllocation() const {
    cerr << "H: br=" << _H.blockRows() << endl;
    cerr << "H: bc=" << _H.blockCols() << endl;
    cerr << "H, nnzb: " << _H.numNonZeroBlocks() << endl;

    cerr << "H: r=" << _H.rows() << endl;
    cerr << "H: c=" << _H.cols() << endl;
    cerr << "H, nnz: " << _H.numNonZeros() << endl;
  }

  void Solver::allocateWorkspace() {
    // we determine the block layout of the linear system
    // and reassign the indices based on the ordering in _active_variables
    _variable_layout.clear();
    std::vector<FactorBase*>& active_factors_level = _active_factors[this->currentLevel()];
    _variable_layout.resize(_active_variables.size());
    for (size_t i = 0; i < _active_variables.size(); ++i) {
      VariableBase* var   = _active_variables[i];
      var->_hessian_index = i;
      _variable_layout[i] = var->perturbationDim();
    }

    // 4. make the layout of H and b
    _H = SparseBlockMatrix(_variable_layout, _variable_layout);
    _b = SparseBlockMatrix(_variable_layout, std::vector<int>(1, 1));
    // 4. populate the target indices of all factors with the right
    //   H and b blocks
    for (auto it = active_factors_level.begin(); it != active_factors_level.end(); ++it) {
      FactorBase* factor = *it;
      factor->clearTargetBlocks();

      int nvars = factor->numVariables();
      for (int r = 0; r < nvars; ++r) {
        VariableBase* row_var = factor->variable(r);
        if (row_var->_hessian_index < 0) {
          continue;
        }

        MatrixBlockBase* bblock = _b.blockAt(row_var->_hessian_index, 0, true);
        factor->setRHSTargetBlock(r, bblock);

        for (int c = r; c < nvars; ++c) {
          VariableBase* col_var = factor->variable(c);
          if (col_var->_hessian_index < 0) {
            continue;
          }

          // we need to decide of a block is transposed or not
          // depending on the hessian indices
          MatrixBlockBase* hblock = 0;
          bool is_transposed      = false;
          if (row_var->_hessian_index <= col_var->_hessian_index) {
            hblock = _H.blockAt(row_var->_hessian_index, col_var->_hessian_index, true);
          } else {
            hblock        = _H.blockAt(col_var->_hessian_index, row_var->_hessian_index, true);
            is_transposed = true;
          }
          factor->setHTargetBlock(r, c, hblock, is_transposed);
        }
      }
    }
    SparseBlockLinearSolverPtr _linear_solver = param_linear_solver.value();

    _linear_solver->setStructureChanged();
  }

  void Solver::assignRobustifiers() {
    std::vector<FactorBase*>& active_factors_level = _active_factors[this->currentLevel()];
    for (auto it = active_factors_level.begin(); it != active_factors_level.end(); ++it) {
      FactorBase* factor = *it;
      for (size_t i = 0; i < param_robustifier_policies.size(); ++i) {
        RobustifierPolicyBasePtr policy = param_robustifier_policies.value(i);
        if (!policy) {
          continue;
        }
        RobustifierBasePtr r = policy->getRobustifier(factor);
        if (r) {
          factor->setRobustifier(r.get());
          break;
        }
      }
    }
  }

  bool Solver::bindFactors() {
    IdVariablePtrContainer& variables = _graph->variables();
    IdFactorPtrContainer& factors     = _graph->factors();

    int num_vars = 0;
    for (auto it = factors.begin(); it != factors.end(); ++it) {
      FactorBase* f = const_cast<FactorBase*>(it.value());
      num_vars += f->bind(variables);
    }
    if (num_vars > 0) {
      std::cerr << "Solver::bindFactors|WARNING, there are [ " << num_vars
                << " ] dangling variables, no optimization possible. Either: \n";
      std::cerr << "\t- instantiate the variables in the graph" << std::endl;
      std::cerr << "\t- set the create_on_bind parameter to true" << std::endl;
      return false;
    }
    return true;
  }

  void Solver::prepareForCompute() {
    assert(param_linear_solver.value() && "Solver::prepareForCompute|linear solver not in");
    assert(_graph && "Solver::prepareForCompute|graph not set");
    if (!bindFactors()) {
      return;
    }
    computeActiveFactors();
    SolverBase::prepareForCompute();
  }

  void Solver::prepareForNewLevel() {
    assignRobustifiers();
    computeActiveVariables();
    computeOrdering();
    allocateWorkspace();
    SolverBase::prepareForNewLevel();
  }

  bool Solver::computeMarginalCovariance(MatrixBlockVector& covariance_matricies_,
                                         const VariablePairVector& variables_) {
    assert(param_linear_solver.value() && "Solver::computeMarginalCovariance|linear solver not in");
    assert(_graph && "Solver::computeMarginalCovariance|no graph set");
    if (_iteration_stats.empty()) {
      throw std::runtime_error("Solver::computeMarginalCovariance|need to call compute() ");
    }

    SparseBlockLinearSolverPtr _linear_solver = param_linear_solver.value();
    SparseBlockMatrix x(_H.blockRowDims(), _H.blockColDims());
    covariance_matricies_.clear();
    covariance_matricies_.reserve(variables_.size());
    // tg compute blocks to be inverted based on the variables hessian indices
    std::vector<IntPair> block_structure;
    block_structure.reserve(variables_.size());
    for (const VariablePair& vp : variables_) {
      block_structure.emplace_back(vp.first->_hessian_index, vp.second->_hessian_index);
    }
    // tg sort blocks
    std::sort(block_structure.begin(),
              block_structure.end(),
              [](const IntPair& a, const IntPair& b) -> bool {
                return a.second < b.second || (a.second == b.second && a.first < b.first);
              });
    // compute block inverse
    if (!_linear_solver->computeBlockInverse(x, block_structure)) {
      return false;
    }

    for (const VariablePair& vp : variables_) {
      MatrixBlockBase* cov_block =
        x.blockRelease(vp.first->_hessian_index, vp.second->_hessian_index);
      if (!cov_block) {
        throw std::runtime_error("Solver::computeMarginalCovariance | no block found, maybe "
                                 "you ask two times the same variables covariance");
      }
      covariance_matricies_.emplace_back(cov_block);
    }

    return true;
  }

  void Solver::extractFisherInformationBlocks(MatrixBlockVector& information_matricies_,
                                              const VariablePairVector& variables_) {
    assert(param_linear_solver.value() &&
           "Solver::extractFisherInformationBlocks|linear solver not in");
    assert(_graph && "Solver::extractFisherInformationBlocks|no graph set");
    if (_iteration_stats.empty()) {
      throw std::runtime_error("Solver::extractFisherInformationBlocks|need to call compute() ");
    }
    information_matricies_.clear();
    information_matricies_.reserve(variables_.size());
    for (const VariablePair& vp : variables_) {
      MatrixBlockBase* info_block =
        _H.blockRelease(vp.first->_hessian_index, vp.second->_hessian_index);
      if (!info_block) {
        throw std::runtime_error("Solver::extractFisherInformationBlocks | no block found, maybe "
                                 "you ask two times the same variables covariance");
      }
      information_matricies_.emplace_back(info_block);
    }
  }

  bool Solver::updateChi(IterationStats& istat) {
    SystemUsageCounter::tic();
    istat.iteration      = _current_iteration;
    bool chi_only        = true;
    istat.num_inliers    = 0;
    istat.num_outliers   = 0;
    istat.chi_inliers    = 0;
    istat.chi_outliers   = 0;
    istat.num_suppressed = 0;
    istat.chi_normalized = 0;
    // tg I have to do this to get the proper index in the vector of factors stats
    size_t current_level = this->currentLevel();
    // tg get proper initial index in factors stats
    int factor_idx                                 = 0;
    istat.level                                    = current_level;
    std::vector<FactorBase*>& active_factors_level = _active_factors[current_level];
    FactorStatsVector& factor_level_stats          = _factor_stats[current_level];
    for (auto it = active_factors_level.begin(); it != active_factors_level.end(); ++it) {
      FactorBase* outer_factor = *it;
      outer_factor->setBegin();
      FactorBase* factor;
      while (outer_factor->getNext(factor)) {
        if (factor->variablesTainted()) {
          factor->compute(chi_only);
        }

        factor_level_stats[factor_idx] = factor->stats();
        FactorStats& mstat             = factor_level_stats[factor_idx];
        ++factor_idx;
        switch (mstat.status) {
          case FactorStats::Status::Inlier:
            istat.num_inliers++;
            istat.chi_inliers += mstat.chi;
            istat.chi_normalized += mstat.kernel_chi;
            break;
          case FactorStats::Status::Kernelized:
            istat.num_outliers++;
            istat.chi_outliers += mstat.chi;
            istat.chi_normalized += mstat.kernel_chi;
            break;
          case FactorStats::Status::Suppressed:
            // ds: fallthrough to return false intended?
            // gg: this happens when a point is outside the camera, or in general
            //     when the error function fails
            istat.num_suppressed++;
            break;
          case FactorStats::Status::Disabled:
            break;
          default:
            return false;
        }
      }
      istat.t_extra += SystemUsageCounter::toc();
    }
    for (auto it : _active_variables) {
      it->untaint();
    }
    return true;
  }

  bool Solver::buildQuadraticForm(IterationStats& istat) {
    _H.setZero();
    _b.setZero();

    istat.reset();
    istat.iteration = _current_iteration;
    // tg I have to do this to get the proper index in the vector of factors stats
    // tg I have to do this to get the proper index in the vector of factors stats
    size_t current_level = this->currentLevel();
    // tg get proper initial index in factors stats
    int factor_idx                                 = 0;
    istat.level                                    = current_level;
    std::vector<FactorBase*>& active_factors_level = _active_factors[current_level];
    FactorStatsVector& factor_level_stats          = _factor_stats[current_level];
    SystemUsageCounter::tic();
    for (auto it = active_factors_level.begin(); it != active_factors_level.end(); ++it) {
      FactorBase* outer_factor = *it;
      FactorBase* factor;
      outer_factor->setBegin();
      while (outer_factor->getNext(factor)) {
        factor->compute();
        factor_level_stats[factor_idx] = factor->stats();
        FactorStats& mstat             = factor_level_stats[factor_idx];
        ++factor_idx;
        switch (mstat.status) {
          case FactorStats::Status::Inlier:
            istat.num_inliers++;
            istat.chi_inliers += mstat.chi;
            istat.chi_normalized += mstat.chi;
            break;
          case FactorStats::Status::Kernelized:
            istat.num_outliers++;
            istat.chi_outliers += mstat.chi;
            istat.chi_normalized += mstat.kernel_chi;
            break;
          case FactorStats::Status::Suppressed:
            istat.num_suppressed++;
            break;
          case FactorStats::Status::Disabled:
            break;
          default:
            return false;
        }
      }
    }
    for (auto it : _active_variables) {
      it->untaint();
    }

    istat.t_linearize += SystemUsageCounter::toc();
    return true;
  }

  bool Solver::solveQuadraticForm(IterationStats& istat) {
    SystemUsageCounter::tic();
    SparseBlockLinearSolverPtr _linear_solver = param_linear_solver.value();
    _linear_solver->setCoefficientsChanged();
    _linear_solver->compute();
    istat.t_solve += SystemUsageCounter::toc();
    if (_linear_solver->status() != SparseBlockLinearSolver::SolutionGood) {
      std::cerr << "Solver::solveQuadraticForm|WARNING, solver error [ "
                << _linear_solver->stringStatus() << " ]\n";
      return false;
    }
    return true;
  }

  void Solver::applyPerturbation(IterationStats& istat) {
    SystemUsageCounter::tic();
    SparseBlockLinearSolverPtr _linear_solver = param_linear_solver.value();

    for (auto it = _active_variables.begin(); it != _active_variables.end(); ++it) {
      VariableBase* var = *it;
      int hessian_index = var->_hessian_index;
      if (hessian_index < 0) {
        continue;
      }
      const MatrixBlockBase* update = _linear_solver->x().blockAt(hessian_index, 0);
      var->applyPerturbationRaw(update->storage());
    }
    istat.t_update += SystemUsageCounter::toc();

    // ia update chi
    updateChi(istat);
  }

  void Solver::getDiagonal(std::vector<float>& diagonal) const {
    _H.getDiagonal(diagonal);
  }

  void Solver::setDiagonal(const std::vector<float>& diagonal) {
    _H.setDiagonal(diagonal);
  }

  void Solver::getPerturbation(std::vector<float>& dx) const {
    const SparseBlockLinearSolverPtr _linear_solver = param_linear_solver.value();
    _linear_solver->x().getDenseVector(dx);
  }

  void Solver::getRHS(std::vector<float>& b) const {
    _b.getDenseVector(b);
  }

  void Solver::push() {
    for (auto it = _active_variables.begin(); it != _active_variables.end(); ++it) {
      (*it)->push();
    }
  }

  void Solver::pop() {
    for (auto it = _active_variables.begin(); it != _active_variables.end(); ++it) {
      (*it)->pop();
    }
  }

  void Solver::discardTop() {
    for (auto it = _active_variables.begin(); it != _active_variables.end(); ++it) {
      (*it)->discardTop();
    }
  }
} // namespace srrg2_solver
