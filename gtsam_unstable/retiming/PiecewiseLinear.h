/**
 * @file PiecewiseLinear.h
 * @brief A piecewise linear used to define conditionals.
 * @author Gerry Chen
 * @date Sept 2023
 */

#pragma once

#include <Eigen/Core>

namespace gtsam {

/**
 * @brief A piecewise linear used to define conditionals.
 *
 * We will represent a piecewise linear with vectors m (slope), b (y-intercept),
 * and xc (segmentation locations "cuts").
 */
struct PiecewiseLinear {
  using Vec = Eigen::VectorXd;

  Vec m, b, xc;

  /// @brief Evaluate the piecewise linear at a point x
  double evaluate(double x) {
    auto i =
        std::distance(xc.begin(), std::lower_bound(xc.begin(), xc.end(), x));
    return m(i) * x + b(i);
  }

  /// @brief The number of segments in the piecewise linear
  auto rows() const { return m.rows(); }
};

}  // namespace gtsam
