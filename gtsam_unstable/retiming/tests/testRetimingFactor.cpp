/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testRetimingFactor.cpp
 *  @brief  Unit tests for RetimingFactor
 *  @author Gerry Chen
 **/

#include <gtsam_unstable/retiming/RetimingFactor.h>

#include <numeric>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

#include <gtsam_unstable/retiming/RetimingFactorGraph.h>
#include <gtsam_unstable/retiming/RetimingConditional.h>
#include <gtsam_unstable/retiming/RetimingBayesNet.h>

using namespace std;
using namespace gtsam;

using Linears = LinearConstraint::Linears;

Linears LinConstr(std::initializer_list<double> a, double b) {
  Vector ret(a.size() + 1);
  std::copy(a.begin(), a.end(), ret.begin());
  ret(a.size()) = b;
  return ret.transpose();
}

Linears Stack(std::initializer_list<Linears> list) {
  auto total_rows =
      std::accumulate(list.begin(), list.end(), 0,
                      [](int acc, const Linears& l) { return acc + l.rows(); });

  Linears ret(total_rows, list.begin()->cols());
  int row = 0;
  for (const auto& l : list) {
    ret.middleRows(row, l.rows()) = l;
    row += l.rows();
  }
  return ret;
}

/* ************************************************************************* */
TEST(RetimingFactor, ConstructFromGraph) {
  Key x = 0, y = 1, z = 2;

  RetimingFactorGraph factors;
  factors.push_back(
      RetimingFactor::Objective({y}, RetimingObjective::Greedy()));
  // factors.push_back(RetimingFactor::Objective({x, z}, PiecewiseQuadratic{}));
  factors.push_back(
      RetimingFactor::Equality({x, y, z}, LinConstr({1, 2, 3}, 0.2)));
  factors.push_back(RetimingFactor::Equality({x, z}, LinConstr({8, 9}, 0.5)));
  factors.push_back(
      RetimingFactor(/*keys*/ {x, z},
                     /*Obj*/
                     RetimingObjectives{RetimingObjective::Greedy()},
                     /*Eq*/
                     Stack({
                         // Eq
                         LinConstr({1.1, 1.2}, 1.3),
                         LinConstr({1.4, 1.5}, 1.6),
                     }),
                     /*Ineq*/
                     Stack({
                         LinConstr({2.1, 2.2}, 2.3),
                         LinConstr({2.4, 2.5}, 2.6),
                     }),
                     false));
  factors.push_back(RetimingFactor::Equality({z}, LinConstr({5}, 0.1)));
  factors.push_back(RetimingFactor::Inequality({y}, LinConstr({6}, 0.3)));
  factors.push_back(
      RetimingFactor::Inequality({z, x}, LinConstr({2, 7}, -0.3)));

  RetimingFactor actual(factors, false, false);

  RetimingFactor expected(
      {x, y, z},
      RetimingObjectives{RetimingObjective::Greedy(),  // PiecewiseQuadratic{},
                         RetimingObjective::Greedy()},
      Stack({
          LinConstr({1, 2, 3}, 0.2),
          LinConstr({8, 0, 9}, 0.5),
          LinConstr({1.1, 0, 1.2}, 1.3),
          LinConstr({1.4, 0, 1.5}, 1.6),
          LinConstr({0, 0, 5}, 0.1),
      }),
      Stack({
          LinConstr({2.1, 0, 2.2}, 2.3),
          LinConstr({2.4, 0, 2.5}, 2.6),
          LinConstr({0, 6, 0}, 0.3),
          LinConstr({7, 0, 2}, -0.3),
      }),
      false, false);

  EXPECT(expected.equals(actual, 1e-9));
}

/* ************************************************************************* */
/*      Suite of tests testing removal of redundant constraints              */
/* ************************************************************************* */

#define TEST_MATRIX_EQUALITY(func, input, expected, checkInfeasibility) \
  {                                                                     \
    Matrix actual = input;                                              \
    func(actual, checkInfeasibility);                                   \
    EXPECT(assert_equal(expected, actual, 1e-9));                       \
  }

#define TEST_MATRIX_INFEASIBILITY(func, input)               \
  {                                                          \
    Matrix actual = input;                                   \
    CHECK_EXCEPTION(func(actual, true), std::runtime_error); \
  }

auto removeRedundantEqualitiesInplace =
    RetimingFactor::removeRedundantEqualitiesInplace;

TEST(RetimingFactor, RemoveEqualityRedundancies_Basic3x4Matrix) {
  Matrix Ab(3, 4);
  Ab << 1, 2, 3, 4,  //
      0, 0, 0, 5,    //
      0, 0, 0, 0;
  Matrix Ab_expected = Ab;
  TEST_MATRIX_EQUALITY(removeRedundantEqualitiesInplace, Ab, Ab_expected,
                       false);
  TEST_MATRIX_INFEASIBILITY(removeRedundantEqualitiesInplace, Ab);
}

TEST(RetimingFactor, RemoveEqualityRedundancies_Basic5x4Matrix) {
  Matrix Ab(5, 4);
  Ab << 1, 2, 3, 4,  //
      0, 5, 6, 7,    //
      0, 0, 1, 8,    //
      0, 0, 0, 0,    //
      0, 0, 0, 0;

  Matrix Ab_expected(4, 4);
  Ab_expected << Ab.topRows(4);

  TEST_MATRIX_EQUALITY(removeRedundantEqualitiesInplace, Ab, Ab_expected, true);
}

TEST(RetimingFactor, RemoveEqualityRedundancies_InfeasibleMatrix) {
  Matrix Ab(3, 4);
  Ab << 1, 2, 3, 4,  //
      0, 0, 0, 5,    //
      0, 0, 0, 10;

  TEST_MATRIX_INFEASIBILITY(removeRedundantEqualitiesInplace, Ab);
}

TEST(RetimingFactor, RemoveEqualityRedundancies_NoRedundantRows) {
  Matrix Ab(3, 4);
  Ab << 1, 2, 3, 4,  //
      5, 6, 7, 8,    //
      9, 10, 11, 12;

  Matrix Ab_expected(3, 4);
  Ab_expected << -10.3440804327886, -11.794185166357096, -13.244289899925591,
      -14.694394633494087,                                      //
      0, 0.94720444555663, 1.89440889111326, 2.84161333666989,  //
      0, 0, 0, 0;

  TEST_MATRIX_EQUALITY(removeRedundantEqualitiesInplace, Ab, Ab_expected, true);
}

TEST(RetimingFactor, RemoveRedundantConstraints1) {
  // First start with a relatively large one where we don't expect the
  // inequalities to change at all
  Matrix Ab(3, 4);
  Ab << 1, 2, 3, 4,  //
      5, 6, 7, 8,    //
      9, 10, 11, 12;

  Matrix Ab_expected(3, 4);
  Ab_expected << -10.3440804327886, -11.794185166357096, -13.244289899925591,
      -14.694394633494087,                                      //
      0, 0.94720444555663, 1.89440889111326, 2.84161333666989,  //
      0, 0, 0, 0;

  Matrix Cd(1, 4);
  Cd << 1, 2, 3, 4;

  RetimingFactor actual({0, 1, 2, 3}, RetimingObjectives{}, Ab, Cd);

  RetimingFactor expected({0, 1, 2, 3}, RetimingObjectives{}, Ab_expected, Cd,
                          false, false);

  EXPECT(expected.equals(actual, 1e-9));
}

TEST(RetimingFactor, RemoveRedundantConstraints2) {
  // Next do a single-variable elimination where we expect the inequalities to
  // be reduced
  Matrix Ab(2, 2);
  Ab << 1, 2,  //
      3, 4;
  Matrix Ab_exp(2, 2);
  Ab_exp << -3.162277660168379, -4.427188724235731,  //
      0, -0.632455532033676;

  Matrix Cd(6, 2);
  Cd << 1, 2,  // x < 2    (inactive)
      -1, 3,   // x > -3   (inactive)
      3, 4,    // x < 0.75 (inactive)
      12, 6,   // x < 0.5  (active)
      -5, -2,  // x > 0.4  (active)
      1, 7;    // x < 7    (inactive)
  Matrix Cd_exp(2, 2);
  Cd_exp << 1, 0.5,  // x < 0.5
      -1, -0.4;      // x > 0.4

  RetimingFactor actual({0}, RetimingObjectives{}, Ab, Cd, true, false);
  RetimingFactor expected({0}, RetimingObjectives{}, Ab_exp, Cd_exp, false,
                          false);
  EXPECT(expected.equals(actual, 1e-9));

  // Infeasible equality constraint
  CHECK_EXCEPTION(RetimingFactor({0}, RetimingObjectives{}, Ab, Cd, true, true),
                  std::runtime_error);  // Infeasible
  // Change Ab to be feasible
  Ab << 1, 2, 5, 10;
  Ab_exp << -5.099019513592785, -10.19803902718557, 0, 0;
  actual = RetimingFactor({0}, RetimingObjectives{}, Ab, Cd, true, true);
  expected =
      RetimingFactor({0}, RetimingObjectives{}, Ab_exp, Cd_exp, false, false);
  EXPECT(expected.equals(actual, 1e-9));

  // Change Cd to be in-feasible
  Cd.row(5) << 1, 0.3;
  CHECK_EXCEPTION(RetimingFactor({0}, RetimingObjectives{}, Ab, Cd, true, true),
                  std::runtime_error);  // Infeasible
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
