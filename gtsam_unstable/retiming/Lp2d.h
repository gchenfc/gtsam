/**
 * @file Lp2d.h
 * @brief Utilities for solving 2-dimensional LPs.
 * @author Gerry Chen
 * @date Sept 2023
 */

#pragma once

#include <Eigen/Core>

namespace gtsam {

namespace lp2d {

// Following the convention of LinearConstraints, an inequality is represented
// as a matrix of the form [a, b, c] where ax + by <= c, and multiple
// inequalities are represented by an Nx3 matrix.
using Inequality = Eigen::Matrix<double, 1, 3>;
using Inequalities = Eigen::Matrix<double, Eigen::Dynamic, 3>;

// Scalar inequalities take the form [a, b] where ax <= b, so 2 (forming a
// bound) would be a 2x2 matrix
using ScalarBounds = Eigen::Matrix<double, 2, 2>;
using Point = Eigen::Matrix<double, 1, 2>;

/// @brief Compute the lower and upper bounds on the second variable, returned
/// in the form [1, upperBound; -1, -lowerBound]
/// Uses a naive algorithm of calculating every possible intersection point
/// between pairwise inequalities, checking if they're feasible, and taking the
/// min/max.
ScalarBounds extremalsY(const Inequalities& inequalities);

/// @brief Compute the upper bound on the second variable
inline double argmaxY(const Inequalities& inequalities) {
  return extremalsY(inequalities)(0, 1);
}

/// @brief Compute the lower bound on the second variable
inline double argminY(const Inequalities& inequalities) {
  return -extremalsY(inequalities)(1, 1);
}

/// @brief Starting from one feasible vertex, sort the inequalities (polygon
/// edges) in counterclockwise order and remove the un-used inequalities
/// @returns False if the problem is infeasible
bool sortBoundaries(const Inequalities& input, Inequalities& result);

/// @brief Insert new inequalities while maintaining sorted property
bool insertBoundariesSorted(const Inequalities& inequalities,
                            const Inequalities& new_inequalities,
                            Inequalities& result);

/// @brief Traverse the inequalities in counterclockwise order starting from
/// start_index
/// @returns The index of the "extremal" inequality (the last inequality before
/// the extremal)
int traverseSortedToExtremal(const Inequalities& inequalities, int start_index,
                             bool ccw);

/// @brief Returns the vertex on the given (ccw) side of the given edge
Point nextVertexFromSorted(const Inequalities& inequalities, int edge_index,
                           bool ccw);

/// @brief Compute the intersection points of a line with the inequalities
/// This is a vectorized version of `intersection` below
Eigen::Array<double, Eigen::Dynamic, 2> computeAllIntersections(
    const Inequalities& inequalities, const Inequality& line);

/// @brief Computes the indices of the feasible points
std::pair<int, int> computeFeasiblePointPair(
    const Inequalities& inequalities,
    const Eigen::Array<double, Eigen::Dynamic, 2>& intersections,
    const Inequality& line, double tol = 1e-12);

/// @brief Check if the inequalities are sorted in counterclockwise order
bool isCcw(const Inequality& line1, const Inequality& line2);

/// @brief Compute the intersection of 2 lines
Point intersection(const Inequality& line1, const Inequality& line2,
                   double parallel_tol = 1e-12);

/// @brief Checks whether a point satisfies the inequalities
bool isFeasible(const Inequalities& inequalities, const Point& point,
                double tol = 1e-12);

}  // namespace lp2d
}  // namespace gtsam
