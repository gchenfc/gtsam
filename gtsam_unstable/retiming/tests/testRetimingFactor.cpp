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
  factors.push_back(RetimingFactor::Objective({x, z}, PiecewiseQuadratic{}));
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
                     })));
  factors.push_back(RetimingFactor::Equality({z}, LinConstr({5}, 0.1)));
  factors.push_back(RetimingFactor::Inequality({y}, LinConstr({6}, 0.3)));
  factors.push_back(
      RetimingFactor::Inequality({z, x}, LinConstr({2, 7}, -0.3)));

  RetimingFactor actual(factors);

  RetimingFactor expected(
      {x, y, z},
      RetimingObjectives{RetimingObjective::Greedy(), PiecewiseQuadratic{},
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
      }));

  EXPECT(expected.equals(actual, 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
