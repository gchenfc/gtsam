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

  double evaluate(double x) { return 0; }
};

}  // namespace gtsam
