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

  Solver::Solver() : _mse_last_iter(0), _num_active_factors(0) {
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
  }

  void Solver::allocateStructures() {
    // cerr << "allocateStructures!" << endl;
    // assert(config() && "config not set");
    if (!_structure_changed_flag) {
      return;
    }

    if (!param_linear_solver.value())
      throw std::runtime_error("Solver::allocateStructures|ERROR, no linear solver set");

    param_linear_solver.value()->bindLinearSystem(&_H, &_b);
    SolverBase::allocateStructures();
  }

  void Solver::computeActiveRegion() {
    // we determine the active factors:
    // a factor is active if
    // at least one variable in it is active and no variables are non_active
    // in doing so we reset the indices of all variables encountered
    _active_factors.clear();
    _num_active_factors         = 0;
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
        _active_factors.push_back(f);
        _num_active_factors += f->size();
      }
    }

    // we scan the active factors and populate the
    // active variables (non fixed)
    _active_variables.clear();
    for (auto it = _active_factors.begin(); it != _active_factors.end(); ++it) {
      FactorBase* f = *it;
      assert(f && "active factor null");
      for (int pos = 0; pos < f->numVariables(); ++pos) {
        VariableBase* v = f->variable(pos);
        assert(v && "variable at active factor null");
        if (v->status() == VariableBase::Active && v->_hessian_index < 0) {
          v->_hessian_index = _active_variables.size();
          _active_variables.push_back(v);
        }
      }
    }
  }

  void Solver::computeOrdering() {
    std::vector<IntPair> block_layout;

    for (auto it = _active_factors.begin(); it != _active_factors.end(); ++it) {
      FactorBase* factor = *it;
      assert(factor && "factor null");
      int nvars = factor->numVariables();
      for (int r = 0; r < nvars; ++r) {
        VariableBase* var_r = factor->variable(r);
        assert(var_r && "var_r null");
        if (var_r->_hessian_index < 0) {
          continue;
        }
        int idx_r = var_r->_hessian_index;
        for (int c = r; c < nvars; ++c) {
          VariableBase* var_c = factor->variable(c);
          assert(var_c && "var_c null");
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
    std::vector<int> ordering;
    ordering.resize(_active_variables.size());
    SparseBlockLinearSolverPtr _linear_solver = param_linear_solver.value();
    //    std::cerr << "computing ordering" << std::endl;
    _linear_solver->computeOrderingHint(ordering, block_layout);
    //    std::cerr << "done" << std::endl;
    std::vector<VariableBase*> src(_active_variables);
    // reorder the variables, by reassigning the indices
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
    for (auto it = _active_factors.begin(); it != _active_factors.end(); ++it) {
      FactorBase* factor = *it;
      factor->clearTargetBlocks();
      // cerr << "factor: " << it->first << " nvars: " << factor->numVariables()
      // << endl;

      int nvars = factor->numVariables();
      for (int r = 0; r < nvars; ++r) {
        VariableBase* row_var = factor->variable(r);
        // cerr << "B[" << r << "]: " << row_var;
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
    for (auto it = _active_factors.begin(); it != _active_factors.end(); ++it) {
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
    // cerr << "prepareForCompute!" << endl;
    assert(param_linear_solver.value() && "Solver::prepareForCompute|linear solver not in");
    assert(_graph && "Solver::prepareForCompute|graph not set");

    if (!bindFactors()) {
      return;
    }

    using namespace std;
    // 2. determine the active variables, and clear the hessian indices
    //   populates _active_variables
    computeActiveRegion();

    assignRobustifiers();

    // 3. compute an initial ordering based on
    computeOrdering();

    // 5. we allocate the H workspace
    allocateWorkspace();
    // std::cerr << "workspace allocated!" << std::endl;

    // cerr << "CF "<< config() << endl;
    // cerr << "LS "<< linear_solver.value() << endl;
    // cerr << "LS_CF "<< linear_solver.value()->config() << endl;
    // cerr << "TC "<< termination_criteria.value() << endl;
    // cerr << "TC_CF "<< termination_criteria.value()->config() << endl;
    // cerr << "STATS "<< max_iterations.value() << endl;
    // 6. resize statistics and get ready
    _factor_stats.resize(_num_active_factors);
    SolverBase::prepareForCompute();
  }

  void Solver::compute() {
    prepareForCompute();
    IterationStats sentinel_iter;
    updateChi(sentinel_iter);
    float mse_sentinel  = sentinel_iter.chi_inliers / (sentinel_iter.num_inliers + 1e-8);
    float mse_variation = mse_sentinel - _mse_last_iter;
    if (std::fabs(mse_variation) > param_mse_threshold.value()) {
      SolverBase::compute();
      const IterationStats& last_iter = _iteration_stats.back();
      _mse_last_iter                  = last_iter.chi_inliers / (last_iter.num_inliers + 1e-8);
    }
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

    for (const VariablePair& vp : variables_) {
      MatrixBlockBase* block =
        x.blockAt(vp.second->_hessian_index, vp.second->_hessian_index, true);
      block->setIdentity();
      if (vp.first != vp.second) {
        block = x.blockAt(vp.first->_hessian_index, vp.second->_hessian_index, true);
        block->setZero();
      }
    }

    if (!_linear_solver->computeBlockInverse(x)) {
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
    int factor_idx       = 0;
    istat.level          = this->currentLevel();
    for (auto it = _active_factors.begin(); it != _active_factors.end(); ++it) {
      FactorBase* outer_factor = *it;
      outer_factor->setBegin();
      FactorBase* factor;
      while (outer_factor->getNext(factor)) {
        factor->setCurrentLevel(this->currentLevel());

        if (factor->variablesTainted()) {
          factor->compute(chi_only);
        }

        _factor_stats[factor_idx] = factor->stats();
        FactorStats& mstat        = _factor_stats[factor_idx];
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
    // cerr << "Linearize... ";
    _H.setZero();
    _b.setZero();

    istat.reset();
    istat.iteration = _current_iteration;
    istat.level     = this->currentLevel();
    // std::cerr << "QF_level: " << this->currentLevel() << std::endl;
    SystemUsageCounter::tic();
    int factor_idx = 0;
    for (auto it = _active_factors.begin(); it != _active_factors.end(); ++it) {
      FactorBase* outer_factor = *it;
      FactorBase* factor;
      outer_factor->setBegin();
      while (outer_factor->getNext(factor)) {
        factor->setCurrentLevel(this->currentLevel());
        factor->compute();
        _factor_stats[factor_idx] = factor->stats();
        FactorStats& mstat        = _factor_stats[factor_idx];
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
