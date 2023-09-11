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

// Forward declare
struct RetimingObjective;

/**
 * @brief A piecewise quadratic used to define objectives.
 *
 * The following representation is used for a quadratic:
 *    f(x) = a.x^2 + b.y^2 + c.x.y + d.x + e.y + f
 *    NOT THIS: f(x) = [x'; y']^T * Q * [x'; y']
 *              where Q is a 2x2 matrix.
 * This is convenient because:
 *  - we can represent the "piecewise" part with *vectors* a, b, c, d, e, f to
 *    vectorize calculations
 *  - parametric solution is easy: x'* = -(c.y + d) / (2a)
 *
 * A piecewise quadratic will be represented with vectors a, b, c, d, e, f, and
 * a vector of dividing points x.  Note that for our application it will always
 * be piecewise in x only.
 */
class PiecewiseQuadratic {
 public:
  using shared_ptr = std::shared_ptr<PiecewiseQuadratic>;
  using This = PiecewiseQuadratic;

  using Mat = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
  using Vec = Eigen::Vector<double, Eigen::Dynamic>;

  using Inequalities = LinearConstraint::Linears;
  using Bounds1d = Eigen::Matrix<double, 2, 2>;

  /// Default Constructor
  PiecewiseQuadratic() = default;

  /// Standard Constructor
  PiecewiseQuadratic(const Vec& a, const Vec& b, const Vec& c, const Vec& d,
                     const Vec& e, const Vec& f, const Vec& xc)
      : a_(a), b_(b), c_(c), d_(d), e_(e), f_(f), xc_(xc) {}

  /// Constructor for a single (non-piecewise) quadratic
  PiecewiseQuadratic(double a, double b, double c, double d, double e, double f)
      : a_(Eigen::Vector<double, 1>(a)),
        b_(Eigen::Vector<double, 1>(b)),
        c_(Eigen::Vector<double, 1>(c)),
        d_(Eigen::Vector<double, 1>(d)),
        e_(Eigen::Vector<double, 1>(e)),
        f_(Eigen::Vector<double, 1>(f)),
        xc_(Vec(0)) {}

  /// Constructor from a vector of objectives
  PiecewiseQuadratic(const std::vector<RetimingObjective>& objectives);

  /// Constructor that sums a bunch of piecewise quadratics
  PiecewiseQuadratic(const std::vector<PiecewiseQuadratic>& objectives);

  /// Evaluate the objective for a given x and y
  double evaluate(double x, double y) const;

  /// Solve a parametric, piecewise QP with linear inequalities to obtain a
  /// piecewise linear solution x^*(y) and inequality bounds on the argument y.
  std::pair<PiecewiseLinear, Bounds1d> solveParametric(
      const Inequalities& inequalities) const;

  /// Substitute a solution x^*(y) into the quadratic to obtain a new
  /// piecewise quadratic on y (one variable, so b, c, e = 0).
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

  // Getters
  const Vec& a() const { return a_; }
  const Vec& b() const { return b_; }
  const Vec& c() const { return c_; }
  const Vec& d() const { return d_; }
  const Vec& e() const { return e_; }
  const Vec& f() const { return f_; }
  const Vec& xc() const { return xc_; }

 private:
  // Member variables
  Vec a_, b_, c_, d_, e_, f_, xc_;
};

template <>
struct traits<PiecewiseQuadratic> : public Testable<PiecewiseQuadratic> {};

}  // namespace gtsam
