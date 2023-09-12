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
  Point err(std::numeric_limits<double>::signaling_NaN(),
            std::numeric_limits<double>::signaling_NaN());
  EXPECT(assert_equal(
      err,                                                      //
      intersection(Inequality(1, 0, 0), Inequality(1, 0, 0))))  // colinear
  EXPECT(assert_equal(
      err,                                                         //
      intersection(Inequality(1, 2, 8), Inequality(0.5, 1, 10))))  // parallel
  EXPECT(assert_equal(
      err,                                                      //
      intersection(Inequality(0, 0, 0), Inequality(1, 0, 0))))  // not a line
  // CHECK_EXCEPTION(intersection(Inequality(1, 0, 0), Inequality(1, 0, 0)),
  //                 std::runtime_error);  // colinear
  // CHECK_EXCEPTION(intersection(Inequality(1, 2, 8), Inequality(0.5, 1, 10)),
  //                 std::runtime_error);  // parallel
  // CHECK_EXCEPTION(intersection(Inequality(0, 0, 0), Inequality(1, 0, 0)),
  //                 std::runtime_error);  // not a line
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
  EXPECT(assert_equal(xbnds_expected, xbnds_actual));
}

/* ************************************************************************* */
TEST(Lp2d, sorted_inequalities) {
  Inequalities inequalities(7, 3);
  inequalities << -1, 1, 0.5,  // y <= x + 0.5
      1. / 3, -1, 0,           // y >= x/3
      -1, -1, -0.5,            // y >= 0.5 - x
      1, 1, 1,                 // y <= 1 - x
      1, 0, 0.5,               // x <= 0.5
      -1, 0, 100,              // x >= -100
      0, -1, 100;              // y >= -100

  // Test initial sort
  Inequalities expected(5, 3);
  expected << inequalities.row(0), inequalities.row(2), inequalities.row(1),
      inequalities.row(4), inequalities.row(3);
  Inequalities actual;
  EXPECT(sortBoundaries(inequalities, actual));
  EXPECT(assert_equal(expected, actual));

  // Test inserting new inequalities
  Inequalities new_inequalities(2, 3);
  new_inequalities << 1.1, -1, 0,  // y >= 1.1*x
      1, 0, 10;                    // x <= 10
  expected.resize(4, 3);
  expected << inequalities.row(0), inequalities.row(2), new_inequalities.row(0),
      inequalities.row(3);
  Inequalities actual2;
  EXPECT(insertBoundariesSorted(actual, new_inequalities, actual2));
  EXPECT(assert_equal(expected, actual2));

  // Test inserting infeasible
  new_inequalities << 1, -1, 0,  // y >= x
      1, 0, -10;                 // x <= -10
  EXPECT(!insertBoundariesSorted(actual, new_inequalities, actual2));
}

/* ************************************************************************* */
TEST(Lp2d, sorted_inequalities_open_boundary) {
  Inequalities inequalities(3, 3);
  inequalities << -1, 0, 0,  // x >= 0
      -1, 1, 1,              // y <= 1 + x
      0, -1, 0;              // y >= 0

  // Test initial sort
  Inequalities expected(3, 3);
  expected << inequalities.row(0), inequalities.row(2), inequalities.row(1);
  Inequalities actual;
  EXPECT(sortBoundaries(inequalities, actual));
  EXPECT(assert_equal(expected, actual));

  // Test inserting new inequalities
  Inequalities new_inequalities(1, 3);
  new_inequalities << 0, 1, 2;  // y <= 2

  expected.resize(4, 3);
  expected << inequalities.row(0), inequalities.row(2), new_inequalities.row(0),
      inequalities.row(1);
  Inequalities actual2;
  EXPECT(insertBoundariesSorted(actual, new_inequalities, actual2));
  EXPECT(assert_equal(expected, actual2));
}

/* ************************************************************************* */
TEST(Lp2d, sorted_inequalities_open_boundary2) {
  Inequalities inequalities(4, 3);
  inequalities << -1, 0, 0,  // x >= 0
      -1, 1, 1,              // y <= 1 + x
      0, -1, 0,              // y >= 0
      -1, -1, 5;             // y >= -5 - x

  // Test initial sort
  Inequalities expected(3, 3);
  expected << inequalities.row(0), inequalities.row(2), inequalities.row(1);
  Inequalities actual;
  EXPECT(sortBoundaries(inequalities, actual));
  EXPECT(assert_equal(expected, actual));

  // Test inserting new inequalities
  Inequalities new_inequalities(1, 3);
  new_inequalities << 0, 1, 2;  // y <= 2

  expected.resize(4, 3);
  expected << inequalities.row(0), inequalities.row(2), new_inequalities.row(0),
      inequalities.row(1);
  Inequalities actual2;
  EXPECT(insertBoundariesSorted(actual, new_inequalities, actual2));
  EXPECT(assert_equal(expected, actual2));
}

/* ************************************************************************* */
TEST(Lp2d, traverse_to_extremal) {
  Inequalities inequalities(7, 3);
  inequalities << -1, 1, 0.5,  // y <= x + 0.5
      1. / 3, -1, 0,           // y >= x/3
      -1, -1, -0.5,            // y >= 0.5 - x
      1, 1, 1,                 // y <= 1 - x
      1, 0, 0.5,               // x <= 0.5
      -1, 0, 100,              // x >= -100
      0, -1, 100;              // y >= -100
  // Sort
  Inequalities sorted(5, 3);
  sorted << inequalities.row(0), inequalities.row(2), inequalities.row(1),
      inequalities.row(4), inequalities.row(3);

  //                                                   start, ccw
  EXPECT_LONGS_EQUAL(1, traverseSortedToExtremal(sorted, 0, true));
  EXPECT_LONGS_EQUAL(1, traverseSortedToExtremal(sorted, 1, true));
  EXPECT_LONGS_EQUAL(4, traverseSortedToExtremal(sorted, 2, true));
  EXPECT_LONGS_EQUAL(4, traverseSortedToExtremal(sorted, 3, true));
  EXPECT_LONGS_EQUAL(4, traverseSortedToExtremal(sorted, 4, true));
  EXPECT_LONGS_EQUAL(0, traverseSortedToExtremal(sorted, 0, false));
  EXPECT_LONGS_EQUAL(0, traverseSortedToExtremal(sorted, 1, false));
  EXPECT_LONGS_EQUAL(2, traverseSortedToExtremal(sorted, 2, false));
  EXPECT_LONGS_EQUAL(2, traverseSortedToExtremal(sorted, 3, false));
  EXPECT_LONGS_EQUAL(2, traverseSortedToExtremal(sorted, 4, false));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
