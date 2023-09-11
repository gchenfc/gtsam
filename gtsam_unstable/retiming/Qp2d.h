/**
 * @file Qp2d.h
 * @brief Utilities for solving 2-dimensional parametric, piecewise QPs.
 * @author Gerry Chen
 * @date Sept 2023
 */

#pragma once

#include <Eigen/Core>

#include "PiecewiseLinear.h"
#include "PiecewiseQuadratic.h"

namespace gtsam {

namespace qp2d {

using Inequalities = Eigen::Matrix<double, Eigen::Dynamic, 3>;
using Bounds1d = Inequalities;

/// @brief Parametrically solve a QP with exactly 2 unknowns x, y to return
/// x^*(y) and ymin <= y <= ymax
/// @param[in] objective
/// @param[in] inequalities
/// @param[out] bounds_on_argument bounds on the argument of the objective,
/// which is the projection of the inequality constraints
/// @return the optimal value of the first variable as a function of the second
PiecewiseLinear argmin(const PiecewiseQuadratic& objective,
                       const Inequalities& inequalities,
                       Bounds1d* bounds_on_argument = nullptr);

}  // namespace qp2d
}  // namespace gtsam
