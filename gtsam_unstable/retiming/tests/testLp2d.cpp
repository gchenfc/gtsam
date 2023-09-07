/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testLp2d.cpp
 *  @brief  Unit tests for 2D LP solver
 *  @author Gerry Chen
 **/

#include <gtsam_unstable/retiming/Lp2d.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/VectorSpace.h>  // Matrix traits

using namespace std;
using namespace gtsam;
using namespace lp2d;

/* ************************************************************************* */
TEST(Lp2d, intersection) {
  Inequality line1(1, 0, 0);  // x = 0
  Inequality line2(0, 1, 0);  // y = 0
  EXPECT(assert_equal(Point(0, 0), intersection(line1, line2)));
}

/* ************************************************************************* */
TEST(eliminate, extremals) {
  //
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
