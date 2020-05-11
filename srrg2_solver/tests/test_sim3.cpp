#include <gtest/gtest.h>

#include <srrg_geometry/similiarity.hpp>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>

#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/variables_and_factors/types_projective/instances.h"

const std::string exe_name = "test_sim3";
#define LOG std::cerr << exe_name + "|"

using namespace srrg2_core;
using namespace srrg2_solver;

int main(int argc, char** argv) {
  variables_and_factors_projective_registerTypes();
  solver_registerTypes();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(Sim3Variable, Sim3SolverVariable) {
  // testing variable sim3 and pertubation
  Vector7f pert;
  pert << 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.f;
  const Similiarity3f sim(Similiarity3f::Identity());
  const Similiarity3f sim_pert_euler(geometry3d::tas2s(pert));
  const Similiarity3f result_sim_er = sim * sim_pert_euler;
  const Similiarity3f result_sim_el = sim_pert_euler * sim;

  const Similiarity3f sim_pert_quaternion(geometry3d::v2s(pert));
  const Similiarity3f result_sim_qr = sim * sim_pert_quaternion;
  const Similiarity3f result_sim_ql = sim_pert_quaternion * sim;

  // perturbation euler right
  VariableSim3EulerRight var_er;
  var_er.setZero();
  var_er.applyPerturbation(pert);
  const Vector7f diff_er = geometry3d::s2v(result_sim_er.inverse() * var_er.estimate());
  ASSERT_LT(diff_er(0), 1e-5);
  ASSERT_LT(diff_er(1), 1e-5);
  ASSERT_LT(diff_er(2), 1e-5);
  ASSERT_LT(diff_er(3), 1e-5);
  ASSERT_LT(diff_er(4), 1e-5);
  ASSERT_LT(diff_er(5), 1e-5);
  ASSERT_LT(diff_er(6), 1e-5);

  // perturbation euler left
  VariableSim3EulerLeft var_el;
  var_el.setZero();
  var_el.applyPerturbation(pert);
  const Vector7f diff_el = geometry3d::s2v(result_sim_el.inverse() * var_el.estimate());
  ASSERT_LT(diff_el(0), 1e-5);
  ASSERT_LT(diff_el(1), 1e-5);
  ASSERT_LT(diff_el(2), 1e-5);
  ASSERT_LT(diff_el(3), 1e-5);
  ASSERT_LT(diff_el(4), 1e-5);
  ASSERT_LT(diff_el(5), 1e-5);
  ASSERT_LT(diff_el(6), 1e-5);

  // perturbation quaternion right
  VariableSim3QuaternionRight var_qr;
  var_qr.setZero();
  var_qr.applyPerturbation(pert);
  const Vector7f diff_qr = geometry3d::s2v(result_sim_qr.inverse() * var_qr.estimate());
  ASSERT_LT(diff_qr(0), 1e-5);
  ASSERT_LT(diff_qr(1), 1e-5);
  ASSERT_LT(diff_qr(2), 1e-5);
  ASSERT_LT(diff_qr(3), 1e-5);
  ASSERT_LT(diff_qr(4), 1e-5);
  ASSERT_LT(diff_qr(5), 1e-5);
  ASSERT_LT(diff_qr(6), 1e-5);

  // perturbation quaternion left
  VariableSim3QuaternionLeft var_ql;
  var_ql.setZero();
  var_ql.applyPerturbation(pert);
  const Vector7f diff_ql = geometry3d::s2v(result_sim_ql.inverse() * var_ql.estimate());
  ASSERT_LT(diff_ql(0), 1e-5);
  ASSERT_LT(diff_ql(1), 1e-5);
  ASSERT_LT(diff_ql(2), 1e-5);
  ASSERT_LT(diff_ql(3), 1e-5);
  ASSERT_LT(diff_ql(4), 1e-5);
  ASSERT_LT(diff_ql(5), 1e-5);
  ASSERT_LT(diff_ql(6), 1e-5);
}

TEST(Sim3VariableAD, Sim3SolverVariable) {
  // testing variable sim3 autodiff and pertubation quaternion right only
  VariableSim3QuaternionRightAD var;
  Vector7_<DualValuef> pert;
  DualValuef dv(0.1f, 1);
  pert << dv, dv, dv, dv, dv, dv, dv;
  var.setZero();
  var.applyPerturbationAD(pert);
  Similiarity3_<DualValuef> sim;
  sim.setIdentity();

  const Similiarity3_<DualValuef> result_sim = sim * geometry3d::v2s(pert);
  const Vector7_<DualValuef> diff       = geometry3d::s2v(result_sim.inverse() * var.adEstimate());
  const Vector7_<DualValuef> var_vector = geometry3d::s2v(var.adEstimate());
  // assert the value should be zero in difference vector
  ASSERT_LT(diff(0).value, 1e-5);
  ASSERT_LT(diff(1).value, 1e-5);
  ASSERT_LT(diff(2).value, 1e-5);
  ASSERT_LT(diff(3).value, 1e-5);
  ASSERT_LT(diff(4).value, 1e-5);
  ASSERT_LT(diff(5).value, 1e-5);
  ASSERT_LT(diff(6).value, 1e-5);

  // assert the derivate, should not be absolute zero
  // because they are actually being computed
  ASSERT_FALSE(var_vector(0).derivative == 0.f);
  ASSERT_FALSE(var_vector(1).derivative == 0.f);
  ASSERT_FALSE(var_vector(2).derivative == 0.f);
  ASSERT_FALSE(var_vector(3).derivative == 0.f);
  ASSERT_FALSE(var_vector(4).derivative == 0.f);
  ASSERT_FALSE(var_vector(5).derivative == 0.f);
  ASSERT_FALSE(var_vector(6).derivative == 0.f);
}