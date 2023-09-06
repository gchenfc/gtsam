/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testLinearEquality.cpp
 *  @brief  Unit tests for LinearEquality
 *  @author Duy-Nguyen Ta
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
      RetimingFactor::Equality({x, y, z}, {.A = {1, 2, 3}, .b = 0.2}));
  factors.push_back(
      RetimingFactor::Inequality({x, y}, {.A = {5, 6}, .b = 0.3}));

  auto [actual_cond, actual_joint] = EliminateRetiming(factors, {x});
  // Expect:
  // conditional: x = 0.2 - 2y - 3z, but actually this is just represented by
  // the same equality because it's trivial to solve joint: (1 - 10y - 15z) + 6y
  // <= 0.3
  //        -4y - 15z <= -0.7

  auto expected_cond = factors.at(0);
  auto expected_joint =
      RetimingFactor::Inequality({y, z}, {.A = {-4, -15}, .b = -0.7});

  std::cout << actual_cond << std::endl;
  std::cout << actual_joint << std::endl;

  CHECK(actual_cond);
  CHECK(actual_joint);
  EXPECT(expected_cond->equals(*actual_cond, 0));
  EXPECT(expected_joint->equals(*actual_joint, 0));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
