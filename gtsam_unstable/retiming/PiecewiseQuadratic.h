/**
 * @file PiecewiseQuadratic.h
 * @brief A piecewise quadratic used to define objectives.
 * @author Gerry Chen
 * @date Sept 2023
 *
 * What do we need out of this class?
 *  - Needs to be able to represent a general quadratic of 1 or 2 variables
 *  - Needs to be able to substitute a linear equality constraint
 *  - Needs to be able to eliminate one variable from a QP (propagate the
 *    objective to the other variable)
 *    - Probably easiest to do this by first solving for the variable
 *      (conditional) then substituting
 */

#pragma once

#include <memory>

#include <Eigen/Core>

#include "PiecewiseLinear.h"
#include "utils.h"

namespace gtsam {

constexpr int kNumVars = 2;

// Forward declare
struct RetimingObjective;

/**
 * @brief A piecewise quadratic used to define objectives.
 *
 * The following representation is used for a quadratic:
 *    x' := x - x0
 *    y' := y - y0
 *    f(x) = a * x'^2 + b * x' * y' + c * y'^2
 *    NOT THIS: f(x) = [x'; y']^T * Q * [x'; y']
 *              where Q is a 2x2 matrix.
 * This is convenient because:
 *  - we can represent the "piecewise" part with vectors a, b, c to vectorize
 *    calculations
 *  - global solution is trivial: x=x0 and y=y0
 *  - parametric solution is easy: x'* = -(b / 2a)y'
 *
 * A piecewise quadratic will be represented with vectors x0, y0, a, b, c, and a
 * vector of dividing points x.  Note that for our application it will always be
 * piecewise in x only.
 */
class PiecewiseQuadratic {
 public:
  using shared_ptr = std::shared_ptr<PiecewiseQuadratic>;
  using This = PiecewiseQuadratic;

  using Mat = Eigen::Matrix<double, kNumVars, kNumVars>;
  using Vec = Eigen::Matrix<double, kNumVars, 1>;

  using Inequalities = LinearConstraint::Linears;
  using Bounds1d = Inequalities;

  /// Default Constructor
  PiecewiseQuadratic() = default;

  /// Constructor from a vector of objectives
  PiecewiseQuadratic(const std::vector<RetimingObjective>& objectives);

  /// Constructor that sums a bunch of piecewise quadratics
  PiecewiseQuadratic(const std::vector<PiecewiseQuadratic>& objectives);

  /// Solve a parametric, piecewise QP with linear inequalities to obtain a
  /// piecewise linear solution x^*(y) and inequality bounds on the argument y.
  std::pair<PiecewiseLinear, Bounds1d> solveParametric(
      const Inequalities& inequalities) const;

  /// Substitute a solution x^*(y) into the quadratic to obtain a new
  /// piecewise quadratic on y (one variable).
  PiecewiseQuadratic substitute(const PiecewiseLinear& conditional) const;

  // TODO(gerry): implementation
  PiecewiseQuadratic rekey(const KeyVector& src_keys,
                           const KeyVector& dest_keys) const {
    std::cout << "Warning: Piecewise Quadratic not yet implemented"
              << std::endl;
    return *this;
  }

  // Testable
  void print(const std::string& s = "Piecewise Quadratic",
             const KeyFormatter& formatter = DefaultKeyFormatter) const {
    std::cout << s << " unimplemented\n";
  }
  bool equals(const This& other, double tol = 1e-9) const {
    std::cout << "Warning: Piecewise Quadratic not yet implemented"
              << std::endl;
    return true;
  }

 private:
  // Member variables
  Vec x0_, y0_, a_, b_, c_, xc_;
};

template <>
struct traits<PiecewiseQuadratic> : public Testable<PiecewiseQuadratic> {};

}  // namespace gtsam
