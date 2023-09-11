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
  PiecewiseQuadratic q(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

  EXPECT_DOUBLES_EQUAL(6.0, q.evaluate(0.0, 0.0), 1e-9);
  EXPECT_DOUBLES_EQUAL(11.0, q.evaluate(1.0, 0.0), 1e-9);
  EXPECT_DOUBLES_EQUAL(13.0, q.evaluate(0.0, 1.0), 1e-9);
  EXPECT_DOUBLES_EQUAL(21.0, q.evaluate(1.0, 1.0), 1e-9);
  EXPECT_DOUBLES_EQUAL(3.0, q.evaluate(-1.0, -1.0), 1e-9);
  EXPECT_DOUBLES_EQUAL(17.0, q.evaluate(-2.0, 3.0), 1e-9);
}

TEST(PiecewiseQuadratic, evaluate2) {
  Vector3 a{1, 2, 3};
  Vector3 b{4, 5, 6};
  Vector3 c{7, 8, 9};
  Vector3 d{0.6, 0.5, 0.4};
  Vector3 e{0.3, 0.2, 0.1};
  Vector3 f{0.9, 0.8, 0.7};
  Vector2 xc{10, 11};
  PiecewiseQuadratic q(a, b, c, d, e, f, xc);

  EXPECT_DOUBLES_EQUAL(349.65, q.evaluate(9.9, 3), 1e-9);
  EXPECT_DOUBLES_EQUAL(613.87, q.evaluate(10.1, 4), 1e-9);
  EXPECT_DOUBLES_EQUAL(89.07, q.evaluate(10.9, -2), 1e-9);
  EXPECT_DOUBLES_EQUAL(2019.01, q.evaluate(11.1, 10.2), 1e-9);
}

/* ************************************************************************* */
TEST(PiecewiseQuadratic, sum) {
  PiecewiseQuadratic q1, q2;
  {
    Vector3 a{1, 2, 3};
    Vector3 b{4, 5, 6};
    Vector3 c{7, 8, 9};
    Vector3 d{1.1, 1.2, 1.3};
    Vector3 e{0.6, 0.5, 0.4};
    Vector3 f{0.3, 0.2, 0.1};
    Vector2 xc{10, 11};
    q1 = PiecewiseQuadratic(a, b, c, d, e, f, xc);
  }
  {
    Vector3 a{9, 8, 0.1};
    Vector3 b{1.3, 1.4, 1.9};
    Vector3 c{0.2, 0.3, 0.4};
    Vector3 d{0.5, 0.6, 0.7};
    Vector3 e{0.8, 0.9, 1.0};
    Vector3 f{1.1, 1.2, 1.3};
    Vector2 xc{4, 10.5};
    q2 = PiecewiseQuadratic(a, b, c, d, e, f, xc);
  }

  auto q3_actual = PiecewiseQuadratic({q1, q2});

  // Now check that q_of_y(y) evaluates to the same as q(x_of_y(y), y)
  auto test_with_xy = [&](double x, double y) {
    EXPECT_DOUBLES_EQUAL(q1.evaluate(x, y) + q2.evaluate(x, y),
                         q3_actual.evaluate(x, y), 1e-9);
  };

  for (double x = 3.1; x < 12.5; x += 0.5) {
    for (double y = 0; y < 12; y += 0.75) {
      test_with_xy(x, y);
    }
  }
}

/* ************************************************************************* */
TEST(PiecewiseQuadratic, substitute) {
  Vector3 a{1, 2, 3};
  Vector3 b{4, 5, 6};
  Vector3 c{7, 8, 9};
  Vector3 d{1.1, 1.2, 1.3};
  Vector3 e{0.6, 0.5, 0.4};
  Vector3 f{0.3, 0.2, 0.1};
  Vector2 xc{-1, 1};
  PiecewiseQuadratic q(a, b, c, d, e, f, xc);

  // y = x - 3,  y < -1.5
  // y = -x,     -1.5 < y < 0
  // y = x,      0 < y < 0.5
  // y = -x + 2, 0.5 < y  (discontinuity at 0.5)
  Vector4 m{1, -1, 1, -1};
  Vector4 b2{3, 0, 0, 2};
  Vector3 yc2{-1.5, 0, 0.5};
  PiecewiseLinear x_of_y{m, b2, yc2};

  auto q_of_y = q.substitute(x_of_y);

  // First check that q is only a function of 1 variable
  EXPECT((q_of_y.b().array() == 0).all());
  EXPECT((q_of_y.c().array() == 0).all());
  EXPECT((q_of_y.e().array() == 0).all());

  // Now check that q_of_y(y) evaluates to the same as q(x_of_y(y), y)
  auto test_with_y = [&](double y) {
    auto x = x_of_y.evaluate(y);
    auto expected = q.evaluate(x, y);
    // std::cout << "Trying " << y << "\tx = " << x
    //           << "\texpected = " << expected << std::endl;
    auto actual = q_of_y.evaluate(y, 0);
    EXPECT_DOUBLES_EQUAL(expected, actual, 1e-9);
    actual = q_of_y.evaluate(y, 10);
    EXPECT_DOUBLES_EQUAL(expected, actual, 1e-9);
    actual = q_of_y.evaluate(y, -10);
    EXPECT_DOUBLES_EQUAL(expected, actual, 1e-9);
  };

  std::vector<double> crossing_points{-4, -2, -1.5, -1, 0, 0.5, 1, 3};
  for (auto y : crossing_points) {
    test_with_y(y - 0.01);
    // test_with_y(y);  // Don't run this because <= vs < isn't well defined
    test_with_y(y + 0.01);
  }
}

/* ************************************************************************* */
TEST(PiecewiseQuadratic, solve_parametric) {
  // See Qp2d_example.ipynb
  // z1: [0.5, 3, 0, 0.5, -0.6, 0.155]
  // z2: [0.8, 1, -1.2, 0, 0, 0]

  Vector2 a{0.5, 0.8};
  Vector2 b{3.0, 1.0};
  Vector2 c{0.0, -1.2};
  Vector2 d{0.5, 0.0};
  Vector2 e{-0.6, 0.0};
  Vector2 f{0.155, 0.0};
  Vector1 xc{0.0};
  PiecewiseQuadratic q(a, b, c, d, e, f, xc);

  Inequalities inequalities(8, 3);
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
  Bounds1d expected_bounds =
      (Bounds1d() << 1, 1, -1, 1).finished();  // -1 <= y <= 1
  EXPECT(assert_equal(expected_bounds, actual_bounds, 1e-9));

  // Construct the expected x^*(y) piecewise linear solution.
  double xc2 = 0.0700194588625913, xc3 = 0.17487850032108213;
  Eigen::Matrix<double, 4, 3> expected_m_b_yc;
  expected_m_b_yc << 0, 0, 0,  // x = 0, for y < 0
      0.75, 0, xc2,            // x = 0.75 * y, for 0 <= y <= xc2
      0, -0.5, xc3,            // x = -0.5, for xc2 < y <= xc3
      0.75, 0, 6.0 / 7.0,      // x = 0.75 * y, for xc3 < y <= 6/7
      -1, 1.5, 999999999;      // x = 1.5 - y, for y > 6/7
  PiecewiseLinear expected_conditional{.m = expected_m_b_yc.col(0),
                                       .b = expected_m_b_yc.col(1),
                                       .xc = expected_m_b_yc.col(2).head<4>()};
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
