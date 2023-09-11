/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testPiecewiseQuadratic.cpp
 *  @brief  Unit tests for piecewise quadratic
 *  @author Gerry Chen
 **/

#include <gtsam_unstable/retiming/PiecewiseQuadratic.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/VectorSpace.h>  // Matrix traits

#include <gtsam_unstable/retiming/PiecewiseLinear.h>

using namespace std;
using namespace gtsam;

using Vec = Eigen::Vector<double, Eigen::Dynamic>;
using Inequalities = Eigen::Matrix<double, Eigen::Dynamic, 3>;
using Bounds1d = Eigen::Matrix<double, 2, 2>;

/* ************************************************************************* */
TEST(PiecewiseQuadratic, evaluate1) {
  // 1x^2 + 2y^2 + 3xy + 4x + 5y + 6
  PiecewiseQuadratic q(1.0, 2.0, 3.0, 4.0, 5.0);

  EXPECT_DOUBLES_EQUAL(6.0, q.evaluate(0.0, 0.0), 1e-9);
  EXPECT_DOUBLES_EQUAL(11.0, q.evaluate(1.0, 0.0), 1e-9);
  EXPECT_DOUBLES_EQUAL(13.0, q.evaluate(0.0, 1.0), 1e-9);
  EXPECT_DOUBLES_EQUAL(21.0, q.evaluate(1.0, 1.0), 1e-9);
  EXPECT_DOUBLES_EQUAL(3.0, q.evaluate(-1.0, -1.0), 1e-9);
  EXPECT_DOUBLES_EQUAL(17.0, q.evaluate(-2.0, 3.0), 1e-9);
}

TEST(PiecewiseQuadratic, evaluate2) {
  Vec a{1, 2, 3};
  Vec b{4, 5, 6};
  Vec c{7, 8, 9};
  Vec d{0.6, 0.5, 0.4};
  Vec e{0.3, 0.2, 0.1};
  Vec f{0.9, 0.8, 0.7};
  Vec xc{10, 11};
  PiecewiseQuadratic q(a, b, c, d, e, f, xc);

  EXPECT_DOUBLES_EQUAL(349.65, q.evaluate(9.9, 3), 1e-9);
  EXPECT_DOUBLES_EQUAL(613.87, q.evaluate(10.1, 4), 1e-9);
  EXPECT_DOUBLES_EQUAL(89.07, q.evaluate(10.9, -2), 1e-9);
  EXPECT_DOUBLES_EQUAL(2019.01, q.evaluate(11.1, 10.2), 1e-9);
}

/* ************************************************************************* */
TEST(PiecewiseQuadratic, substitute) {
  Vec a{1, 2, 3};
  Vec b{4, 5, 6};
  Vec c{7, 8, 9};
  Vec d{1.1, 1.2, 1.3};
  Vec e{0.6, 0.5, 0.4};
  Vec f{0.3, 0.2, 0.1};
  Vec xc{10, 11};
  PiecewiseQuadratic q(a, b, c, d, e, f, xc);

  Vec m{-1.0, -2.0, -3.0};
  Vec b2{0.1, 0.2, 0.3};
  Vec xc2{0.4, 0.5};
  PiecewiseLinear x_of_y { m, b2, xc2 }

  auto q_of_y = q.substitute(x_of_y);

  // First check that q is only a function of 1 variable
  EXPECT((q_of_y.b().array() == 0).all());
  EXPECT((q_of_y.c().array() == 0).all());
  EXPECT((q_of_y.e().array() == 0).all());

  // Now check that q_of_y(y) evaluates to the same as q(x_of_y(y), y)
  auto test_with_y = [&](double y) {
    auto x = x_of_y.evaluate(y);
    auto expected = q.evaluate(x, y);
    auto actual = q_of_y.evaluate(y, 0);
    EXPECT_DOUBLES_EQUAL(expected, actual, 1e-9);
    actual = q_of_y.evaluate(y, 10);
    EXPECT_DOUBLES_EQUAL(expected, actual, 1e-9);
    actual = q_of_y.evaluate(y, -10);
    EXPECT_DOUBLES_EQUAL(expected, actual, 1e-9);
  };

  test_with_y(0.0);
  test_with_y(0.1);
  test_with_y(0.2);

  test_with_y(0.39);
  test_with_y(0.40);
  test_with_y(0.41);

  test_with_y(0.49);
  test_with_y(0.50);
  test_with_y(0.51);

  test_with_y(9.99);
  test_with_y(10.00);
  test_with_y(10.01);

  test_with_y(10.99);
  test_with_y(11.00);
  test_with_y(11.01);

  test_with_y(100);
}

/* ************************************************************************* */
TEST(PiecewiseQuadratic, solve_parametric) {
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

  auto [actual_objective, actual_bounds] = q.solveParametric(inequalities);

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
  // Substitute x^*(y) into the objective function to obtain the propagated
  // objective function
  PiecewiseQuadratic expected_objective = q.substitute(expected_conditional);

  // Compare to the expected objective function
  EXPECT(assert_equal(expected_objective, actual_objective, 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
