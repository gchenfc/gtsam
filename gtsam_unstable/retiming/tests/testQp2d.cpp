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

#include <gtsam_unstable/retiming/Lp2d.h>
#include <gtsam_unstable/retiming/PiecewiseLinear.h>
#include <gtsam_unstable/retiming/PiecewiseQuadratic.h>

using namespace std;
using namespace gtsam;

using Inequalities = Eigen::Matrix<double, Eigen::Dynamic, 3>;
using Bounds1d = Eigen::Matrix<double, 2, 2>;

/* ************************************************************************* */
TEST(Qp2d, min_single_quadratic) {
  // See Qp2d_example.ipynb
  // ~~~z1: [0.5, 3, 0, 0.5, -0.6, 0.155]~~~
  // z2: [0.8, 1, -1.2, 0, 0, 0]

  Vector2 a{0.5, 0.8};
  Vector2 b{3.0, 1.0};
  Vector2 c{0.0, -1.2};
  Vector2 d{0.5, 0.0};
  Vector2 e{-0.6, 0.0};
  Vector2 f{0.155, 0.0};
  Vector1 xc{0.0};
  PiecewiseQuadratic q2(a, b, c, d, e, f, xc);
  qp2d::Quadratic q1d = q2.C().bottomRows<1>();
  PiecewiseQuadratic q(q1d(0), q1d(1), q1d(2), q1d(3), q1d(4), q1d(5));

  Inequalities inequalities(9, 3);
  inequalities << 1.0, 0.0, 1.0,  // x <= 1
      -1.0, 0.0, 1.0,             // x >= -1
      0.0, 1.0, 1.0,              // y <= 1
      0.0, -1.0, 1.0,             // y >= -1
      1.0, 1.0, 1.5,              // x + y <= 1.5
      -1.0, -1.0, 1.5,            // x + y >= -1.5
      1.0, -1.0, 1.5,             // x - y <= 1.5
      -1.0, 1.0, 1.5,             // x - y >= -1.5
      -1, 0, 0;                   // x >= 0, so we only get z2

  Inequalities sorted;
  CHECK(lp2d::sortBoundaries(inequalities, sorted));
  auto actual_objective = qp2d::min(q1d, sorted);

  // Construct the expected x^*(y) piecewise linear solution.
  //    x = 0,        for y < 0
  //    x = 0.75 * y, for 0 <= y <= 6/7
  //    x = 1.5 - y,  for 6/7 < y <= 1
  // Quadratic is:
  //    0.8 * x^2 + 1 * y^2 - 1.2 * x * y
  // Objectives are:
  //    1 * y^2 + 0 * y + 0,      for -1 < y < 0
  //    0.55 * y^2 + 0 * y + 0,   for 0 <= y <= 6/7
  //    3 * y^2 - 4.2 * y + 1.8,  for 6/7 < y <= 1
  // Note that we for this function, we explicitly return the -1, +1 bounds
  PiecewiseQuadratic1d expected_objective{
      .C = (Matrix(3, 3) << 1, 0, 0,  //
            0.55, 0, 0,               //
            3, -4.2, 1.8)
               .finished(),
      .xc = (Vector(4) << -1, 0, 6.0 / 7.0, 1).finished()};

  // Compare to the expected objective function
  EXPECT(assert_equal(expected_objective, actual_objective, 1e-9));
}

/* ************************************************************************* */
TEST(Qp2d, min) {
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

  Bounds1d actual_bounds;
  auto actual_objective = qp2d::min(q, inequalities, &actual_bounds);

  // Compare to expected bounds on y
  Bounds1d expected_bounds = (Bounds1d() << 1, 1, -1, 1)  // -1 <= y <= 1
                                 .finished();
  EXPECT(assert_equal(expected_bounds, actual_bounds, 1e-9));

  // Objectives are:
  //    1 * y^2 + 0 * y + 0,      for -1 < y < 0
  //    0.55 * y^2 + 0 * y + 0,   for 0 <= y <= 0.0700194588625913
  //    3 * y^2 - 0.6 * y + 0.03, for 0.07001945886259 <= y <= 0.174878500321082
  //    0.55 * y^2 + 0 * y + 0,   for 0.17487850032108213 <= y <= 6/7
  //    3 * y^2 - 4.2 * y + 1.8,  for 6/7 < y <= 1
  // Note that we for this function, we explicitly return the -1, +1 bounds
  PiecewiseQuadratic1d expected_objective{
      .C = (Matrix(5, 3) << 1, 0, 0,  //
            0.55, 0, 0,               //
            3, -0.6, 0.03,            //
            0.55, 0, 0,               //
            3, -4.2, 1.8)
               .finished(),
      .xc = (Vector(4) << 0, 0.0700194588625913, 0.17487850032108213, 6.0 / 7.0)
                .finished()};

  // Compare to the expected objective function
  EXPECT(assert_equal(expected_objective, actual_objective, 1e-9));
}

TEST(Qp2d, min_2) {
  std::cout << "************ TEST OF INTEREST **************" << std::endl;
  //         Inequality Constraint: 1*x2 + 0*x3 <= 0.3044
  //         Inequality Constraint: -0.96*x2 + 2*x3 <= 0.1
  // combined objective is C:
  //        1        0        0     -0.6        0     0.31
  //  5.34028        0        0 -2.28403        0 0.473351
  // xc:
  // 0.194

  Inequalities inequalities(2, 3);
  inequalities << 1, 0, 0.3044,  //
      -0.96, 2, 0.1;             //
  PiecewiseQuadratic q(Vector2{1.0, 5.34028}, Vector2{0.0, 0.0},
                       Vector2{0.0, 0.0}, Vector2{-0.6, -2.28403},
                       Vector2{0.0, 0.0}, Vector2{0.31, 0.473351},
                       Vector1{0.194});

  Bounds1d actual_bounds;
  auto actual_objective = qp2d::min(q, inequalities, &actual_bounds);

  // Compare to expected bounds on y <= 0.196112
  double inf = std::numeric_limits<double>::infinity();
  Bounds1d expected_bounds = (Bounds1d() << 1, 0.196112, -1, -inf).finished();
  EXPECT(assert_equal(expected_bounds, actual_bounds, 1e-9));

  // Compare to the expected objective function
  // https://photos.app.goo.gl/8ejSwagsMhBSiLy5A
  PiecewiseQuadratic1d expected_objective{
      .C = (Matrix(2, 3) << 0, 0, 0.2291319216,  //
            23.1782986111, -7.07622569444, 0.7692155382)
               .finished(),
      .xc = Vector1(0.1526476514)};
  EXPECT(assert_equal(expected_objective, actual_objective, 1e-6));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
