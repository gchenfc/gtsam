/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testPiecewiseLinear.cpp
 *  @brief  Unit tests for piecewise linear
 *  @author Gerry Chen
 **/

#include <gtsam_unstable/retiming/PiecewiseLinear.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/VectorSpace.h>  // Matrix traits

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(PiecewiseQuadratic, evaluate1) {
  // y = m.x + b
  Matrix m_b_xc(4, 3);
  m_b_xc << 1.0, 2.0, 3.0,  //
      4.0, 5.0, 4.0,        //
      -1.0, -2.0, 5.0,      //
      0.0, 0.0, 9999999;
  PiecewiseLinear l(m_b_xc.col(0), m_b_xc.col(1), m_b_xc.col(2));

  EXPECT_DOUBLES_EQUAL(2.0, l.evaluate(0.0), 1e-9);
  EXPECT_DOUBLES_EQUAL(4.9, l.evaluate(2.9), 1e-9);
  EXPECT_DOUBLES_EQUAL(17.4, l.evaluate(3.1), 1e-9);
  EXPECT_DOUBLES_EQUAL(20.6, l.evaluate(3.9), 1e-9);
  EXPECT_DOUBLES_EQUAL(-6.1, l.evaluate(4.1), 1e-9);
  EXPECT_DOUBLES_EQUAL(-6.9, l.evaluate(4.9), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.0, l.evaluate(5.1), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.0, l.evaluate(1000), 1e-9);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
