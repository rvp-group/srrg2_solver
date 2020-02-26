#pragma once
#include "srrg_solver/solver_core/ad_variable.h"
#include "variable_point3.h"

namespace srrg2_solver {

using namespace srrg2_core;

// point variable with AD
struct VariablePoint3AD: public ADVariable_<VariablePoint3> {
  //    typedef typename ADVariable_<VariablePoint3>::ADPerturbationVectorType ADPerturbationVectorType;
  typedef ADVariable_ <VariablePoint3> ADVariableType;
  typedef VariablePoint3 VariableType;
  typedef typename ADVariableType::ADPerturbationVectorType ADPerturbationVectorType;
  typedef typename ADVariableType::ADEstimateType ADEstimateType;

  virtual ~VariablePoint3AD() {};

  virtual void setZero() override {
    setEstimate(Vector3f::Zero());
  }

  virtual void applyPerturbationAD(const ADPerturbationVectorType& ad_pert) override {
    _ad_estimate += ad_pert;
  }
};
}
