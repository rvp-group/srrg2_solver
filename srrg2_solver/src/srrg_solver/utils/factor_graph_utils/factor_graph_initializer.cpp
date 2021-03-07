#include "factor_graph_initializer.h"
#include "srrg_solver/variables_and_factors/types_2d/instances.h"
#include "srrg_solver/variables_and_factors/types_3d/instances.h"
#include "srrg_solver/variables_and_factors/types_projective/instances.h"

namespace srrg2_solver {

  void FactorGraphInitializer::setGraph(FactorGraphInterface& graph_) {
    _graph = &graph_;
    _entries.clear();
    _queue = FactorGraphVisitEntryQueue();
    // populate the entry structure to hold the visit
    // populate the queue with all fixed variables
    for (auto it = _graph->variables().begin(); it != _graph->variables().end(); ++it) {
      VariableBase* variable    = it.value();
      if (variable->status() == VariableBase::NonActive)
        continue;
      VariableVisitEntry* entry = new VariableVisitEntry(variable);
      if (variable->status() == VariableBase::Fixed) {
        entry->cost = 0;
        _queue.push(entry);
      } else {
        entry->cost = -1;
      }
      _entries.add(entry);
    }
    //std::cerr << "queue.size()" << _queue.size() << std::endl;
  }

  void FactorGraphInitializer::updateGraph() {
    // populate the queue with all variables that are fixed
    // and all the ones that have been initialized
    
    for (auto it = _graph->variables().begin(); it != _graph->variables().end(); ++it) {
      VariableBase* variable    = it.value();
      if (variable->status() == VariableBase::NonActive)
        continue;
      VariableVisitEntry* entry = _entries.at(variable->graphId());
      if (entry) {
        if (entry->cost>=0)
          _queue.push(entry);
      } else {
        VariableVisitEntry* entry = new VariableVisitEntry(variable);
        if (variable->status() == VariableBase::Fixed) {
          entry->cost = 0;
          _queue.push(entry);
        } else {
          entry->cost=-1;
        }
        _entries.add(entry);
      }
    }
    //std::cerr << "queue.size()" << _queue.size() << std::endl;
  }

  void FactorGraphInitializer::compute() {
    while (!_queue.empty()) {
      VariableVisitEntry* e = _queue.top();
      // std::cerr << "pop: " << e->variable->graphId() << " " << e->cost << std::endl;
      _queue.pop();
      VariableBase* v = e->variable;
      auto f_it       = _graph->lowerFactor(v);
      auto f_upper    = _graph->upperFactor(v);
      for (; f_it != f_upper; ++f_it) {
        int v_pos          = -1;
        FactorBase* f = f_it->second;
        if (! f->enabled())
          continue;
        for (int i = 0; i < f->numVariables(); ++i) {
          if (f->variable(i) == v) {
            e->var_pos = i;
            v_pos      = i;
            break;
          }
        }

        // scan all variables
        for (int nv_pos = 0; nv_pos < f->numVariables(); ++nv_pos) {
          if (nv_pos == v_pos)
            continue;
          VariableBase* nv       = f->variable(nv_pos);
          VariableVisitEntry* ne = _entries.at(nv->graphId());
          // std::cerr << "\t nv:" << nv->graphId() << std::endl;
          assert(ne && "bookkeeping error 2");
          // if variable already initialized initialized
          if (ne->cost >= 0)
            continue;
          if (initVariable(nv)) {
            ne->cost = e->cost + 1;
            _queue.push(ne);
            // std::cerr << "push: " << ne->variable->graphId() << " " << ne->cost << std::endl;
          }
        }
      }
    }
  }

  bool FactorGraphInitializer::initVariable(VariableBase* variable) {
    for (auto it = _rules.begin(); it != _rules.end(); ++it) {
      if ((*it)->init(variable))
        return true;
    }
    return false;
  }

  bool FactorGraphInitializer::isInit(VariableBase* v) {
    VariableVisitEntry* ne = _entries.at(v->graphId());
    if (! ne)
      return false;
    return ne->cost >= 0;
  }

  struct SE2toSE2InitializerRule : public FactorGraphInitializerRule_<VariableSE2Base> {
    SE2toSE2InitializerRule(FactorGraphInitializer* initializer_) :
      FactorGraphInitializerRule_<VariableSE2Base>(initializer_) {
    }

    bool doInit(VariableSE2Base* v) override {
      FactorGraphInterface* _graph = _initializer->_graph;
      auto f_it                    = _graph->lowerFactor(v);
      auto f_end                   = _graph->upperFactor(v);
      for (; f_it != f_end; ++f_it) {
        FactorBase* f = f_it->second;
        SE2PosePoseGeodesicErrorFactor* this_factor =
          dynamic_cast<SE2PosePoseGeodesicErrorFactor*>(f);
        if (!this_factor)
          continue;
        bool direct             = true;
        VariableSE2Base* root_v = this_factor->variables().at<0>();
        if (root_v == v) {
          root_v = this_factor->variables().at<1>();
          direct = false;
        }
        if (!_initializer->isInit(root_v))
          continue;

        if (direct) {
          v->setEstimate(root_v->estimate() * this_factor->measurement());
        } else {
          v->setEstimate(root_v->estimate() * this_factor->measurement().inverse());
        }
        //std::cerr << "se2 pose_pose init" << root_v->graphId() << " " << v->graphId() << std::endl;
        return true;
      }
      return false;
    }
  };

  struct SE2toPoint2InitializerRule : public FactorGraphInitializerRule_<VariablePoint2> {
    SE2toPoint2InitializerRule(FactorGraphInitializer* initializer_) :
      FactorGraphInitializerRule_<VariablePoint2>(initializer_) {
    }

    bool doInit(VariablePoint2* v) override {
      FactorGraphInterface* _graph = _initializer->_graph;
      auto f_it                    = _graph->lowerFactor(v);
      auto f_end                   = _graph->upperFactor(v);
      for (; f_it != f_end; ++f_it) {
        FactorBase* f = f_it->second;
        SE2PosePointErrorFactor* this_factor =
          dynamic_cast<SE2PosePointErrorFactor*>(f);
        if (!this_factor) {
          continue;
        }
        VariableSE2Base* root_v = this_factor->variables().at<0>();
        if (!_initializer->isInit(root_v))
          continue;
        v->setEstimate(root_v->estimate() * this_factor->measurement());
        //std::cerr << "se2 pose_point init" << root_v->graphId() << " " << v->graphId() << std::endl;
        return true;
      }
      return false;
    }
  };

  struct SE2toPoint2BearingInitializerRule : public FactorGraphInitializerRule_<VariablePoint2> {
    SE2toPoint2BearingInitializerRule(FactorGraphInitializer* initializer_) :
      FactorGraphInitializerRule_<VariablePoint2>(initializer_) {
    }

    bool doInit(VariablePoint2* v) override {
      FactorGraphInterface* _graph = _initializer->_graph;
      auto f_it                    = _graph->lowerFactor(v);
      auto f_end                   = _graph->upperFactor(v);
      std::list<SE2PosePointBearingErrorFactor*> active_factors;

      for (; f_it != f_end; ++f_it) {
        FactorBase* f = f_it->second;
        SE2PosePointBearingErrorFactor* this_factor =
          dynamic_cast<SE2PosePointBearingErrorFactor*>(f);
        if (!this_factor) {
          continue;
        }
        VariableSE2Base* root_v = this_factor->variables().at<0>();
        if (!_initializer->isInit(root_v))
          continue;
        active_factors.push_back(this_factor);
      }
      if (active_factors.size() < 2)
        return false;
      // do a triangulation with the factors
      Matrix2f H = Matrix2f::Zero();
      Vector2f b = Vector2f::Zero();
      Vector2f x = v->estimate();
      for (auto it = active_factors.begin(); it != active_factors.end(); ++it) {
        SE2PosePointBearingErrorFactor* f = *it;
        VariableSE2Base* pose                    = f->variables().at<0>();
        float theta                              = f->measurement()(0);
        Vector2f n        = pose->estimate().linear() * Vector2f(cos(theta), sin(theta));
        n                 = Vector2f(n.y(), -n.x());
        const Vector2f& p = pose->estimate().translation();
        float e           = n.dot(x - p);
        H += n * n.transpose();
        b += n * e;
      }
      x -= (H).ldlt().solve(b);
      v->setEstimate(x);
      // todo: check for consistency of solution, otherwise disable variable
      //std::cerr << "se2 pose point bearinf :" << x.transpose() << std::endl;
      return true;
    }
  };

  struct SE3toSE3InitializerRule : public FactorGraphInitializerRule_<VariableSE3Base> {
    SE3toSE3InitializerRule(FactorGraphInitializer* initializer_) :
      FactorGraphInitializerRule_<VariableSE3Base>(initializer_) {
    }

    bool doInit(VariableSE3Base* v) override {
      FactorGraphInterface* _graph = _initializer->_graph;
      auto f_it                    = _graph->lowerFactor(v);
      auto f_end                   = _graph->upperFactor(v);
      for (; f_it != f_end; ++f_it) {
        FactorBase* f = f_it->second;
        SE3PosePoseGeodesicErrorFactor* this_factor =
          dynamic_cast<SE3PosePoseGeodesicErrorFactor*>(f);
        if (!this_factor)
          continue;
        bool direct             = true;
        VariableSE3Base* root_v = this_factor->variables().at<0>();
        if (root_v == v) {
          root_v = this_factor->variables().at<1>();
          direct = false;
        }
        if (!_initializer->isInit(root_v))
          continue;

        if (direct) {
          v->setEstimate(root_v->estimate() * this_factor->measurement());
        } else {
          v->setEstimate(root_v->estimate() * this_factor->measurement().inverse());
        }
        //std::cerr << "se3 pose_pose init" << root_v->graphId() << " " << v->graphId() << std::endl;
        return true;
      }
      return false;
    }
  };

  struct Sim3toSim3InitializerRule : public FactorGraphInitializerRule_<VariableSim3Base> {
    Sim3toSim3InitializerRule(FactorGraphInitializer* initializer_) :
      FactorGraphInitializerRule_<VariableSim3Base>(initializer_) {
    }

    bool doInit(VariableSim3Base* v) override {
      FactorGraphInterface* _graph = _initializer->_graph;
      auto f_it                    = _graph->lowerFactor(v);
      auto f_end                   = _graph->upperFactor(v);
      for (; f_it != f_end; ++f_it) {
        FactorBase* f = f_it->second;
        Sim3PosePoseErrorFactorAD* this_factor =
          dynamic_cast<Sim3PosePoseErrorFactorAD*>(f);
        if (!this_factor)
          continue;
        bool direct             = true;
        VariableSim3Base* root_v = this_factor->variables().at<0>();
        if (root_v == v) {
          root_v = this_factor->variables().at<1>();
          direct = false;
        }
        if (!_initializer->isInit(root_v))
          continue;

        if (direct) {
          v->setEstimate(root_v->estimate() * this_factor->measurement());
        } else {
          v->setEstimate(root_v->estimate() * this_factor->measurement().inverse());
        }
        //std::cerr << "se3 pose_pose init" << root_v->graphId() << " " << v->graphId() << std::endl;
        return true;
      }
      return false;
    }
  };

  struct SE3toPoint3InitializerRule : public FactorGraphInitializerRule_<VariablePoint3> {
    SE3toPoint3InitializerRule(FactorGraphInitializer* initializer_) :
      FactorGraphInitializerRule_<VariablePoint3>(initializer_) {
    }

    bool doInit(VariablePoint3* v) override {
      FactorGraphInterface* _graph = _initializer->_graph;
      auto f_it                    = _graph->lowerFactor(v);
      auto f_end                   = _graph->upperFactor(v);
      for (; f_it != f_end; ++f_it) {
        FactorBase* f = f_it->second;
        SE3PosePointErrorFactor* this_factor =
          dynamic_cast<SE3PosePointErrorFactor*>(f);
        if (!this_factor)
          continue;
        VariableSE3Base* root_v = this_factor->variables().at<0>();
        if (!_initializer->isInit(root_v))
          continue;
        v->setEstimate(root_v->estimate() * this_factor->measurement());
        //std::cerr << "se3 pose_point init" << root_v->graphId() << " " << v->graphId() << std::endl;
        return true;
      }
      return false;
    }
  };

  struct SE3toPoint3OffsetInitializerRule : public FactorGraphInitializerRule_<VariablePoint3> {
    SE3toPoint3OffsetInitializerRule(FactorGraphInitializer* initializer_) :
      FactorGraphInitializerRule_<VariablePoint3>(initializer_) {
    }

    bool doInit(VariablePoint3* v) override {
      FactorGraphInterface* _graph = _initializer->_graph;
      auto f_it                    = _graph->lowerFactor(v);
      auto f_end                   = _graph->upperFactor(v);
      for (; f_it != f_end; ++f_it) {
        FactorBase* f = f_it->second;
        SE3PosePointOffsetErrorFactor* this_factor =
          dynamic_cast<SE3PosePointOffsetErrorFactor*>(f);
        if (!this_factor)
          continue;
        VariableSE3Base* offset_v = this_factor->variables().at<2>();
        if (!_initializer->isInit(offset_v))
          continue;
        VariableSE3Base* root_v = this_factor->variables().at<0>();
        if (!_initializer->isInit(root_v))
          continue;

        v->setEstimate(root_v->estimate() * offset_v->estimate() * this_factor->measurement());
        //std::cerr << "se3 pose_point_offset init" << root_v->graphId() << " " << v->graphId() << std::endl;
        return true;
      }
      return false;
    }
  };


  struct SE3toPoint3OmniBAInitializerRule : public FactorGraphInitializerRule_<VariablePoint3> {
    SE3toPoint3OmniBAInitializerRule(FactorGraphInitializer* initializer_) :
      FactorGraphInitializerRule_<VariablePoint3>(initializer_) {
    }

    bool doInit(VariablePoint3* v) override {
      FactorGraphInterface* _graph = _initializer->_graph;
      auto f_it                    = _graph->lowerFactor(v);
      auto f_end                   = _graph->upperFactor(v);
      std::list<SE3PosePointOmniBAErrorFactor*> active_factors;

      for (; f_it != f_end; ++f_it) {
        FactorBase* f = f_it->second;
        SE3PosePointOmniBAErrorFactor* this_factor =
          dynamic_cast<SE3PosePointOmniBAErrorFactor*>(f);
        if (!this_factor) {
          continue;
        }
        VariableSE3Base* root_v = this_factor->variables().at<0>();
        if (!_initializer->isInit(root_v))
          continue;
        active_factors.push_back(this_factor);
      }
      if (active_factors.size() < 3)
        return false;
      // do a triangulation with the factors
      Matrix3f H = Matrix3f::Zero();
      Vector3f b = Vector3f::Zero();
      Vector3f x = v->estimate();
      for (auto it = active_factors.begin(); it != active_factors.end(); ++it) {
        SE3PosePointOmniBAErrorFactor* f = *it;
        VariableSE3Base* pose                    = f->variables().at<0>();
        VariableSE3Base* offset                  = f->variables().at<2>();
        Vector3f d        = pose->estimate().linear()*offset->estimate().linear()*f->measurement();
        Matrix3f J=geometry3d::skew(d);
        const Vector3f& p = pose->estimate()*offset->estimate().translation();
        Vector3f e        = J*(x - p);
        H += J.transpose() * J;
        b += J.transpose() * e;
      }
      x -= (H).ldlt().solve(b);
      v->setEstimate(x);
      // todo: check for consistency of solution, otherwise disable variable
      //std::cerr << "se3 pose point omni ba:" << x.transpose() << std::endl;
      return true;
    }
  };

  FactorGraphInitializer::FactorGraphInitializer() {
    _rules.push_back(FactorGraphInitializerRulePtr(new SE2toSE2InitializerRule(this)));
    _rules.push_back(FactorGraphInitializerRulePtr(new SE2toPoint2InitializerRule(this)));
    _rules.push_back(FactorGraphInitializerRulePtr(new SE2toPoint2BearingInitializerRule(this)));
    _rules.push_back(FactorGraphInitializerRulePtr(new SE3toSE3InitializerRule(this)));
    _rules.push_back(FactorGraphInitializerRulePtr(new SE3toPoint3InitializerRule(this)));
    _rules.push_back(FactorGraphInitializerRulePtr(new SE3toPoint3OffsetInitializerRule(this)));
    _rules.push_back(FactorGraphInitializerRulePtr(new SE3toPoint3OmniBAInitializerRule(this)));
    _rules.push_back(FactorGraphInitializerRulePtr(new Sim3toSim3InitializerRule(this)));
  }

} // namespace srrg2_solver
