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

std::string str(const Eigen::MatrixXd& mat) {
  std::stringstream ss;
  ss << "(" << mat << ")";
  return ss.str();
}

/* ************************************************************************* */
TEST(Lp2d, intersection) {
  Inequality line1(1, 0, 0);  // x = 0
  Inequality line2(0, 1, 0);  // y = 0
  EXPECT(assert_equal(Point(0, 0), intersection(line1, line2)));

  auto checkIfSolSatisfies = [](const Inequality& line1,
                                const Inequality& line2, double tol = 1e-12) {
    auto sol = intersection(line1, line2);
    return (abs(line1.head<2>().dot(sol) - line1(2)) < tol) &&
           (abs(line2.head<2>().dot(sol) - line2(2)) < tol);
  };

  auto expect_solves = [&](const Inequality& line1, const Inequality& line2) {
    if (!(checkIfSolSatisfies(line1, line2))) {
      result_.addFailure(Failure(name_, __FILE__, __LINE__,
                                 str(line1) + " intersect " + str(line2) +
                                     " is not " +
                                     str(intersection(line1, line2))));
    }
  };

  expect_solves(Inequality(1, 2, 3), Inequality(4, 5, 6));
  expect_solves(Inequality(1, 8.7, -2.5), Inequality(-0.2, -0.5, 5));
  expect_solves(Inequality(1, 0, 3), Inequality(2, 0.1, 6));

  // Bad cases
  CHECK_EXCEPTION(intersection(Inequality(1, 0, 0), Inequality(1, 0, 0)),
                  runtime_error);  // colinear
  CHECK_EXCEPTION(intersection(Inequality(1, 2, 8), Inequality(0.5, 1, 10)),
                  runtime_error);  // parallel
  CHECK_EXCEPTION(intersection(Inequality(0, 0, 0), Inequality(1, 0, 0)),
                  runtime_error);  // not a line
}

/* ************************************************************************* */
TEST(Lp2d, is_feasible) {
  // Example LP:  https://photos.app.goo.gl/FpafEDQe1iTT7qhJ8
  Inequalities inequalities(7, 3);
  inequalities << -1, 1, 0.5,  // y <= x + 0.5
      1, 1, 1,                 // y <= 1 - x
      -1, -1, -0.5,            // y >= 0.5 - x
      1. / 3, -1, 0,           // y >= x/3
      1, 0, 0.5,               // x <= 0.5
      -1, 0, 100,              // x >= -100
      0, -1, 100;              // y >= -100

  EXPECT(isFeasible(inequalities, Point(0.5, 0.5)));
  EXPECT(isFeasible(inequalities, Point(0.2, 0.5)));
  EXPECT(isFeasible(inequalities, Point(0.2, 0.65)));
  EXPECT(isFeasible(inequalities, Point(0.3, 0.2)));
  EXPECT(isFeasible(inequalities, Point(0.4, 0.4 / 3.)));
  EXPECT(isFeasible(inequalities, Point(0.1, 0.4)));

  EXPECT(!isFeasible(inequalities, Point(0.501, 0.5)));
  EXPECT(!isFeasible(inequalities, Point(0.501, 0.48)));
  EXPECT(!isFeasible(inequalities, Point(0.5, 0.501)));
  EXPECT(!isFeasible(inequalities, Point(0.4, 0.601)));
  EXPECT(!isFeasible(inequalities, Point(0.2, 0.701)));
  EXPECT(!isFeasible(inequalities, Point(0.2, 0.299)));
  EXPECT(!isFeasible(inequalities, Point(0.4, 0.132)));

  EXPECT(isFeasible(inequalities, Point(0.5, 0.5)));
  EXPECT(isFeasible(inequalities, Point(0.5, 0.48)));
  EXPECT(isFeasible(inequalities, Point(0.4, 0.6)));
  EXPECT(isFeasible(inequalities, Point(0.2, 0.7)));
  EXPECT(isFeasible(inequalities, Point(0.2, 0.3)));
  EXPECT(isFeasible(inequalities, Point(0.4, 0.400001 / 3.)));
}

/* ************************************************************************* */
TEST(Lp2d, extremals) {
  // Example LP:  https://photos.app.goo.gl/FpafEDQe1iTT7qhJ8
  Inequalities inequalities(7, 3);
  inequalities << -1, 1, 0.5,  // y <= x + 0.5
      1, 1, 1,                 // y <= 1 - x
      -1, -1, -0.5,            // y >= 0.5 - x
      1. / 3, -1, 0,           // y >= x/3
      1, 0, 0.5,               // x <= 0.5
      -1, 0, 100,              // x >= -100
      0, -1, 100;              // y >= -100
  // Sol for y: 0.375 <= y <= 0.75
  // Sol for x: 0 <= x <= 0.5

  auto ybnds_actual = extremalsY(inequalities);
  auto ybnds_expected = (ScalarBounds() << 1, 0.75, -1, -0.125).finished();
  EXPECT(assert_equal(ybnds_expected, ybnds_actual));

  // We can also test the x direction by just permuting the first 2 columns
  Inequalities inequalities_x(7, 3);
  inequalities_x << inequalities.col(1), inequalities.col(0),
      inequalities.col(2);
  auto xbnds_actual = extremalsY(inequalities_x);
  auto xbnds_expected = (ScalarBounds() << 1, 0.5, -1, 0.0).finished();
  EXPECT(assert_equal(ybnds_expected, ybnds_actual));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
