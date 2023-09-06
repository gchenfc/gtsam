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

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

#include <gtsam_unstable/retiming/RetimingFactorGraph.h>
#include <gtsam_unstable/retiming/RetimingConditional.h>
#include <gtsam_unstable/retiming/RetimingBayesNet.h>

using namespace std;
using namespace gtsam;

using Linears = RetimingFactor::Linears;

/* ************************************************************************* */
TEST(RetimingFactor, ConstructFromGraph) {
  Key x = 0, y = 1, z = 2;

  RetimingFactorGraph factors;
  factors.push_back(
      RetimingFactor::Objective({y}, RetimingObjective::Greedy()));
  factors.push_back(RetimingFactor::Objective({x, z}, PiecewiseQuadratic{}));
  factors.push_back(
      RetimingFactor::Equality({x, y, z}, {.A = {1, 2, 3}, .b = 0.2}));
  factors.push_back(RetimingFactor::Equality({x, z}, {.A = {8, 9}, .b = 0.5}));
  factors.push_back(
      RetimingFactor(/*keys*/ {x, z},
                     /*Obj*/
                     RetimingObjectives{RetimingObjective::Greedy()},
                     /*Eq*/
                     Linears{
                         // Eq
                         {.A = {1.1, 1.2}, .b = 1.3},
                         {.A = {1.4, 1.5}, .b = 1.6},
                     },
                     /*Ineq*/
                     Linears{
                         {.A = {2.1, 2.2}, .b = 2.3},
                         {.A = {2.4, 2.5}, .b = 2.6},
                     }));
  factors.push_back(RetimingFactor::Equality({z}, {.A = {5}, .b = 0.1}));
  factors.push_back(RetimingFactor::Inequality({y}, {.A = {6}, .b = 0.3}));
  factors.push_back(
      RetimingFactor::Inequality({z, x}, {.A = {2, 7}, .b = -0.3}));

  RetimingFactor actual(factors);

  RetimingFactor expected(
      {x, y, z},
      RetimingObjectives{RetimingObjective::Greedy(), PiecewiseQuadratic{},
                         RetimingObjective::Greedy()},
      Linears{
          {.A = {1, 2, 3}, .b = 0.2},
          {.A = {8, 0, 9}, .b = 0.5},
          {.A = {1.1, 0, 1.2}, .b = 1.3},
          {.A = {1.4, 0, 1.5}, .b = 1.6},
          {.A = {0, 0, 5}, .b = 0.1},
      },
      Linears{
          {.A = {2.1, 0, 2.2}, .b = 2.3},
          {.A = {2.4, 0, 2.5}, .b = 2.6},
          {.A = {0, 6, 0}, .b = 0.3},
          {.A = {7, 0, 2}, .b = -0.3},
      });

  EXPECT(expected.equals(actual, 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
