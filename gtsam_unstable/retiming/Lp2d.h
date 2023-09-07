/**
 * @file Lp2d.h
 * @brief Utilities for solving 2-dimensional LPs
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
using Point = Eigen::Matrix<double, 1, 2>;

/// @brief Compute the lower and upper bounds on the second variable, returned
/// in the form [1, upperBound; -1, -lowerBound]
Inequalities extremalsY(const Inequalities& inequalities);

/// @brief Compute the upper bound on the second variable
double argmaxY(const Inequalities& inequalities) {
  return extremalsY(inequalities)(0, 1);
}

/// @brief Compute the lower bound on the second variable
double argminY(const Inequalities& inequalities) {
  return -extremalsY(inequalities)(1, 1);
}

/// @brief Compute the intersection of 2 lines
Point intersection(const Inequality& line1, const Inequality& line2);

}  // namespace lp2d
}  // namespace gtsam
