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

#include <gtsam_unstable/retiming/RetimingFactorGraph.h>
#include <gtsam_unstable/retiming/RetimingFactor.h>
#include <gtsam_unstable/retiming/RetimingConditional.h>
#include <gtsam_unstable/retiming/RetimingBayesNet.h>

using namespace std;
using namespace gtsam;

Matrix LinConstr(std::initializer_list<double> a, double b) {
  Vector ret(a.size() + 1);
  std::copy(a.begin(), a.end(), ret.begin());
  ret(a.size()) = b;
  return ret.transpose();
}

/* ************************************************************************* */
TEST(eliminate, eliminate_linear_equality) {
  // GTSAM_EXPORT std::pair<std::shared_ptr<RetimingConditional>,
  //                        std::shared_ptr<RetimingFactor> >
  // EliminateRetiming(const RetimingFactorGraph& factors, const Ordering&
  // keys);

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
  // the same equality because it's trivial to solve joint: (1 - 10y - 15z) + 6y
  // <= 0.3
  //        -4y - 15z <= -0.7

  auto expected_cond = factors.at(0);
  auto expected_joint =
      RetimingFactor::Inequality({y, z}, LinConstr({-4, -15}, -0.7));

  CHECK(actual_cond);
  CHECK(actual_joint);
  EXPECT(expected_cond->equals(*actual_cond, 1e-9));
  EXPECT(expected_joint->equals(*actual_joint, 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
