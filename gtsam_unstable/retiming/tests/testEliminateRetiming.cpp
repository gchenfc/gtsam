/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testEliminateRetiming.cpp
 *  @brief  Unit tests for retiming/eliminate.h
 *  @author Gerry Chen
 **/

#include <gtsam_unstable/retiming/eliminate.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam_unstable/retiming/RetimingFactorGraph.h>
#include <gtsam_unstable/retiming/RetimingFactor.h>
#include <gtsam_unstable/retiming/RetimingConditional.h>
#include <gtsam_unstable/retiming/RetimingBayesNet.h>

using namespace std;
using namespace gtsam;
using gtsam::symbol_shorthand::U;
using gtsam::symbol_shorthand::X;

Matrix LinConstr(std::initializer_list<double> a, double b) {
  Vector ret(a.size() + 1);
  std::copy(a.begin(), a.end(), ret.begin());
  ret(a.size()) = b;
  return ret.transpose();
}

/* ************************************************************************* */
TEST(eliminate, eliminate_linear_equality) {
  Key x = 0, y = 1, z = 2;

  // 1x + 2y + 3z = 0.2
  // 5x + 6y     <= 0.3
  RetimingFactorGraph factors;
  factors.push_back(
      RetimingFactor::Equality({x, y, z}, LinConstr({1, 2, 3}, 0.2)));
  factors.push_back(RetimingFactor::Inequality({x, y}, LinConstr({5, 6}, 0.3)));

  auto [actual_cond, actual_joint] = EliminateRetiming(factors, {x});
  // Expect:
  // conditional: x = 0.2 - 2y - 3z, but actually this is just represented by
  // the same equality because it's trivial to solve
  // joint: (1 - 10y - 15z) + 6y <= 0.3
  //                   -4y - 15z <= -0.7

  auto expected_cond = factors.at(0);
  auto expected_joint =
      RetimingFactor::Inequality({y, z}, LinConstr({-4, -15}, -0.7));

  CHECK(actual_cond);
  CHECK(actual_joint);
  EXPECT(expected_cond->equals(*actual_cond, 1e-9));
  EXPECT(expected_joint->equals(*actual_joint, 1e-9));
}

/* ************************************************************************* */
TEST(eliminate, eliminate_linear_equality_long) {
  // x_{k+1} = 1.03 * x_{k}
  // x_0 = 1.0
  // Expect solution to be x_100 = 1.03^100 = 19.2186319809

  RetimingFactorGraph factors;
  for (int i = 0; i < 100; ++i) {
    factors.push_back(
        RetimingFactor::Equality({X(i), X(i + 1)}, LinConstr({1.03, -1}, 0.0)));
  }
  factors.push_back(RetimingFactor::Equality({X(0)}, LinConstr({1}, 1.0)));

  auto sol = factors.eliminateSequential();
  CHECK(sol);
  auto actual_x100 = sol->at(100);
  CHECK(actual_x100);

  // sol->print("Solution!");
  // actual_x100->print("Final Conditional:");

  auto expected_x100 =
      RetimingConditional::Equality({X(100)}, LinConstr({1}, 19.2186319809));

  EXPECT_LONGS_EQUAL(101, sol->size());
  EXPECT(expected_x100->equals(*sol->at(100), 1e-9));
}

/* ************************************************************************* */
TEST(eliminate, eliminate_linear_inequality1) {
  Key x = 0, y = 1;

  // 5x + 6y     <= 30.0
  // -1x         <= 0
  //    - 1y     <= 0
  RetimingFactorGraph factors;
  factors.push_back(
      RetimingFactor::Inequality({x, y}, LinConstr({5, 6}, 30.0)));
  factors.push_back(RetimingFactor::Inequality({x}, LinConstr({-1}, 0.0)));
  factors.push_back(RetimingFactor::Inequality({y}, LinConstr({-1}, 0.0)));

  auto [actual_cond, actual_joint] = EliminateRetiming(factors, {y});

  // Expect:
  // conditional: lazy eval:  5x + 6y <= 30.0
  //                          -1x     <= 0
  //                             - 1y <= 0
  // joint: 0 <= x <= 6.0
  RetimingFactorGraph exp_cond(factors);
  auto expected_cond = RetimingFactor(exp_cond, {y, x});
  Matrix joint_ineq(2, 2);
  joint_ineq << LinConstr({-1}, 0.0), LinConstr({1}, 6.0);
  auto expected_joint =
      RetimingFactor({x}, RetimingObjectives{}, Matrix::Zero(0, 2), joint_ineq);

  CHECK(actual_cond);
  CHECK(actual_joint);
  EXPECT(expected_cond.equals(*actual_cond, 1e-9));
  EXPECT(expected_joint.equals(*actual_joint, 1e-9));
}

/* ************************************************************************* */
TEST(eliminate, eliminate_linear_inequality_edge_case) {
  Key x = 0, y = 1, z = 2;

  // 1x + 0y + 3z = 0.2
  // 5x + 6y     <= 30.0
  // -1x         <= 0
  //    - 1y     <= 0
  RetimingFactorGraph factors;
  factors.push_back(
      RetimingFactor::Equality({x, y, z}, LinConstr({1, 0, 3}, 0.2)));
  factors.push_back(
      RetimingFactor::Inequality({x, y}, LinConstr({5, 6}, 30.0)));
  factors.push_back(RetimingFactor::Inequality({x}, LinConstr({-1}, 0.0)));
  factors.push_back(RetimingFactor::Inequality({y}, LinConstr({-1}, 0.0)));

  auto [actual_cond, actual_joint] = EliminateRetiming(factors, {y});

  // Expect:
  // conditional: lazy eval:  5x + 6y <= 30.0
  //                          -1x     <= 0
  //                             - 1y <= 0
  // joint: 1x + 3z = 0.2
  //        0 <= x <= 6.0
  RetimingFactorGraph exp_cond;
  exp_cond += factors.at(1), factors.at(2), factors.at(3);
  auto expected_cond = RetimingFactor(exp_cond, {y, x});
  Matrix joint_ineq(2, 3);
  joint_ineq << LinConstr({1, 0}, 6.0), LinConstr({-1, 0}, 0.0);
  auto expected_joint = RetimingFactor({x, z}, RetimingObjectives{},
                                       LinConstr({1, 3}, 0.2), joint_ineq);

  CHECK(actual_cond);
  CHECK(actual_joint);
  EXPECT(expected_cond.equals(*actual_cond, 1e-9));
  EXPECT(expected_joint.equals(*actual_joint, 1e-9));
}

/* ************************************************************************* */
TEST(eliminate, eliminate_inequality_long) {
  // x_{k+1} <= 1.03 * x_{k}
  // x_0 = 1.0
  // Expect solution to be x_100 = 1.03^100 = 19.2186319809

  RetimingFactorGraph factors;
  for (int i = 0; i < 100; ++i) {
    factors.push_back(RetimingFactor::Inequality({X(i), X(i + 1)},
                                                 LinConstr({-1.03, 1}, 0.0)));
  }
  factors.push_back(RetimingFactor::Equality({X(0)}, LinConstr({1}, 1.0)));

  auto sol = factors.eliminateSequential();
  CHECK(sol);
  auto actual_x100 = sol->at(100);
  CHECK(actual_x100);

  auto expected_x100 =
      RetimingConditional::Inequality({X(100)}, LinConstr({1}, 19.2186319809));
  EXPECT_LONGS_EQUAL(101, sol->size());
  EXPECT(expected_x100->equals(*sol->at(100), 1e-9));

  // sol->print("Bayes Net:");
  // actual_x100->print("Final Conditional (Act):");
  // expected_x100->print("Final Conditional (Exp):");
}

/* ************************************************************************* */
TEST(eliminate, eliminate_control_limited_dynamics) {
  // System Dynamics:
  //    x_{k+1} = 0.98 * x_k + 0.5 * u_k
  //    x_0 = 1.0
  //    x_1 <= 0.3
  //    u_k + x_k <= 0.1
  //
  // Expect solution to be:
  //    x0 = 1.0, u0 = -1.36  // So that x1 = 0.3
  //    x1 = 0.3, u1 = -0.2
  //    x2 = 0.194, u2 = -0.094
  //    x3 = 0.14312, u3 = -0.04312
  //    ...
  //    xinf = 0.1 / 1.04 = 0.09615384615, uinf = -0.00384615385

  RetimingFactorGraph factors;
  for (int i = 0; i < 100; ++i) {
    factors.push_back(RetimingFactor::Equality(
        {X(i + 1), X(i), U(i)}, LinConstr({-1.0, 0.98, 0.5}, 0.0)));
    factors.push_back(
        RetimingFactor::Inequality({U(i), X(i)}, LinConstr({1, 1}, 0.1)));
  }
  factors.push_back(RetimingFactor::Equality({X(0)}, LinConstr({1}, 1.0)));
  factors.push_back(RetimingFactor::Inequality({X(1)}, LinConstr({1}, 0.3)));

  Ordering ordering;
  for (int i = 0; i < 100; ++i) {
    ordering.push_back(U(i));
  }
  for (int i = 0; i <= 100; ++i) {
    ordering.push_back(X(i));
  }
  auto bn = factors.eliminateSequential();  // COLAMD
  CHECK(bn);
  auto sol = bn->optimize();
  auto bn1 = factors.eliminateSequential(ordering);
  CHECK(bn1);
  auto sol1 = bn1->optimize();

  // Print Solution nicely
  // auto printSol = [](const ScalarValues& sol) {
  //   for (int i = 0; i < 100; ++i) {
  //     std::cout << "\tx" << i << " = " << sol.at(X(i)) << ", u" << i << " = "
  //               << sol.at(U(i)) << "\n";
  //   }
  //   std::cout << "\tx100 = " << sol.at(X(100)) << "\n";
  // };
  // bn->print("Bayes Net:");
  // traits<ScalarValues>::Print(sol, "SOLUTION:");
  // printSol(sol);
  // bn1->print("Bayes Net:");
  // traits<ScalarValues>::Print(sol1, "SOLUTION:");
  // printSol(sol1);

  EXPECT_DOUBLES_EQUAL(1.0, sol.at(X(0)), 1e-9);
  EXPECT_DOUBLES_EQUAL(-1.36, sol.at(U(0)), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.3, sol.at(X(1)), 1e-9);
  EXPECT_DOUBLES_EQUAL(-0.2, sol.at(U(1)), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.194, sol.at(X(2)), 1e-9);
  EXPECT_DOUBLES_EQUAL(-0.094, sol.at(U(2)), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.14312, sol.at(X(3)), 1e-9);
  EXPECT_DOUBLES_EQUAL(-0.04312, sol.at(U(3)), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.09615384615, sol.at(X(100)), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.00384615385, sol.at(U(99)), 1e-9);

  EXPECT_DOUBLES_EQUAL(1.0, sol1.at(X(0)), 1e-9);
  EXPECT_DOUBLES_EQUAL(-1.36, sol1.at(U(0)), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.3, sol1.at(X(1)), 1e-9);
  EXPECT_DOUBLES_EQUAL(-0.2, sol1.at(U(1)), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.194, sol1.at(X(2)), 1e-9);
  EXPECT_DOUBLES_EQUAL(-0.094, sol1.at(U(2)), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.14312, sol1.at(X(3)), 1e-9);
  EXPECT_DOUBLES_EQUAL(-0.04312, sol1.at(U(3)), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.09615384615, sol1.at(X(100)), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.00384615385, sol1.at(U(99)), 1e-9);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
