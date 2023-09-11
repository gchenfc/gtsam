/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testQp2d.cpp
 *  @brief  Unit tests for 2-variable parametric QP solver Qp2d.h
 *  @author Gerry Chen
 **/

#include <gtsam_unstable/retiming/Qp2d.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/VectorSpace.h>  // Matrix traits

#include <gtsam_unstable/retiming/PiecewiseLinear.h>
#include <gtsam_unstable/retiming/PiecewiseQuadratic.h>

using namespace std;
using namespace gtsam;

using Vec = Eigen::Vector<double, Eigen::Dynamic>;
using Inequalities = Eigen::Matrix<double, Eigen::Dynamic, 3>;
using Bounds1d = Eigen::Matrix<double, 2, 2>;

/* ************************************************************************* */
TEST(Qp2d, solve_parametric) {
  // See Qp2d_example.ipynb
  // z1: [0.5, 3, 0, 0.5, -0.6, 0.155]
  // z2: [0.8, 1, -1.2, 0, 0, 0]

  Vec a{0.5, 0.8};
  Vec b{3.0, 1.0};
  Vec c{0.0, -1.2};
  Vec d{0.5, 0.0};
  Vec e{-0.6, 0.0};
  Vec f{0.155, 0.0};
  Vec xc{0.0};
  PiecewiseQuadratic q(a, b, c, d, e, f);

  Inequalities inequalities(8);
  inequalities << 1.0, 0.0, 1.0,  // x <= 1
      -1.0, 0.0, 1.0,             // x >= -1
      0.0, 1.0, 1.0,              // y <= 1
      0.0, -1.0, 1.0,             // y >= -1
      1.0, 1.0, 1.5,              // x + y <= 1.5
      -1.0, -1.0, 1.5,            // x + y >= -1.5
      1.0, -1.0, 1.5,             // x - y <= 1.5
      -1.0, 1.0, 1.5;             // x - y >= -1.5

  Bounds1d actual_bounds;
  auto actual_conditional = qp2d.argmin(q, inequalities, actual_bounds);

  // Compare to expected bounds on y
  Bounds1d expected_bounds(1, 1, -1, 1);  // -1 <= y <= 1
  EXPECT(assert_equal(expected_bounds, actual_bounds, 1e-9));

  // Construct the expected x^*(y) piecewise linear solution.
  double xc2 = 0.0700194588625913, xc3 = 0.17487850032108213;
  Eigen::Matrix<double, 4, 3> expected_m_b_yc;
  expected_m_b_yc << 0, 0, 0,  // x = 0, for y < 0
      0.75, 0, xc2,            // x = 0.75 * y, for 0 <= y <= xc2
      0, -0.5, xc3,            // x = -0.5, for xc2 < y <= xc3
      0.75, 0, 6.0 / 7.0,      // x = 0.75 * y, for xc3 < y <= 6/7
      -1, 1.5, 999999999;      // x = 1.5 - y, for y > 6/7
  PiecewiseLinear expected_conditional(expected_m_b_yc.col(0),
                                       expected_m_b_yc.col(1),
                                       expected_m_b_yc.col(2).head<4>());

  // Compare to the expected objective function
  EXPECT(assert_equal(expected_conditional, actual_conditional, 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
