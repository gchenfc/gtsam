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
using Bounds1d = Eigen::Matrix<double, 2, 2>;
using Quadratic = Eigen::Matrix<double, 1, 6>;

/// @brief Parametrically solve a QP with exactly 2 unknowns x, y to return
/// joint(f) = min_x f(x, y) and ymin <= y <= ymax
/// @param[in] objective
/// @param[in] inequalities
/// @param[out] bounds_on_argument bounds on the argument of the objective,
/// which is the projection of the inequality constraints
/// @return the optimal value of the first variable as a function of the second
PiecewiseQuadratic1d min(const PiecewiseQuadratic& objective,
                         const Inequalities& inequalities,
                         Bounds1d* bounds_on_argument = nullptr);

/// @brief Parametrically solve a QP with single quadratic (not piecewise)
/// @param[in] objective Single quadratic given by a,b,c,d,e,f
/// @param[in] inequalities The inequality constraints
/// @return 1d piecewise quadratic representing f(x^*(y), y) is a function of y.
/// IMPORTANT: This PiecewiseQuadratic disobeys the traditional convention that
/// xc is 1 smaller than the number of segments.  Instead, we do xc1 is one
/// larger since we want to have the bounds.
PiecewiseQuadratic1d min(const Eigen::Ref<const Quadratic>& objective,
                         const Inequalities& inequalities);

}  // namespace qp2d
}  // namespace gtsam
