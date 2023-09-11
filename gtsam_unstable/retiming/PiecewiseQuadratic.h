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
 * @brief A piecewise quadratic of 1 variable
 *
 * Parameterized by: f(x) = a.x^2 + b.x + c
 * a, b, c are stacked into an (n,3) matrix C, and xc is a vector of size (n-1)
 * denoting the x-coordinates of the segment boundaries
 */
struct PiecewiseQuadratic1d {
  Eigen::Matrix<double, Eigen::Dynamic, 3> C;
  Eigen::VectorXd xc;

  /// @brief Evaluate the piecewise quadratic at a point x
  double evaluate(double x) {
    auto i =
        std::distance(xc.begin(), std::upper_bound(xc.begin(), xc.end(), x));
    return C(i, 0) * x * x + C(i, 1) * x + C(i, 2);
  }

  // Testable
  void print(const std::string& s = "Piecewise Quadratic 1d",
             const KeyFormatter& formatter = DefaultKeyFormatter) const {
    std::cout << s << "C:\n" << C << "\nxc:\n" << xc << std::endl;
  }
  bool equals(const PiecewiseQuadratic1d& other, double tol = 1e-9) const {
    return traits<decltype(C)>::Equals(C, other.C, tol) &&
           traits<decltype(xc)>::Equals(xc, other.xc, tol);
  }
};

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

  using Mat = Eigen::Matrix<double, Eigen::Dynamic, 6>;
  using Vec = Eigen::Vector<double, Eigen::Dynamic>;

  using Inequalities = LinearConstraint::Linears;
  using Bounds1d = Eigen::Matrix<double, 2, 2>;

  /// Default Constructor
  PiecewiseQuadratic() = default;

  /// Copy constructor
  PiecewiseQuadratic(const PiecewiseQuadratic& other) = default;

  /// Standard Constructor
  PiecewiseQuadratic(const Mat& C, const Vec& xc) : C_(C), xc_(xc) {}

  /// Vector Constructor
  PiecewiseQuadratic(const Vec& a, const Vec& b, const Vec& c, const Vec& d,
                     const Vec& e, const Vec& f, const Vec& xc)
      : C_((Mat(a.size(), 6) << a, b, c, d, e, f).finished()), xc_(xc) {}

  /// Constructor for a single (non-piecewise) quadratic
  PiecewiseQuadratic(double a, double b, double c, double d, double e, double f)
      : C_((Mat(1, 6) << a, b, c, d, e, f).finished()), xc_(Vec(0)) {}

  /// Constructor from PiecewiseQuadratic1d, by setting b = c = e = 0
  PiecewiseQuadratic(const PiecewiseQuadratic1d& q1d)
      : C_((Mat(q1d.C.rows(), 6) << q1d.C.col(0), Vec::Zero(q1d.C.rows()),
            Vec::Zero(q1d.C.rows()), q1d.C.col(1), Vec::Zero(q1d.C.rows()),
            q1d.C.col(2))
               .finished()),
        xc_(q1d.xc) {}

  /// Constructor from a vector of objectives
  PiecewiseQuadratic(const std::vector<RetimingObjective>& objectives);

  /// Constructor that sums a bunch of piecewise quadratics
  PiecewiseQuadratic(const std::vector<PiecewiseQuadratic>& objectives);

  /// Find which piecewise segment x is in
  auto findIndex(double x) const {
    return std::distance(xc_.begin(),
                         std::lower_bound(xc_.begin(), xc_.end(), x));
  }

  /// Evaluate the objective for a given x and y
  double evaluate(double x, double y) const;

  /// Solve a parametric, piecewise QP with linear inequalities to obtain a
  /// piecewise linear solution x^*(y) and inequality bounds on the argument y.
  std::pair<PiecewiseQuadratic, Bounds1d> solveParametric(
      const Inequalities& inequalities) const;

  /// Substitute a piecewise linear solution x^*(y) into the quadratic to obtain
  /// a new piecewise quadratic on y (one variable, so b, c, e = 0).
  /// We might not even need this entire function because it's more efficient
  /// for solveParametric to return the objective function rather than the
  /// conditional, but it's good practice I guess
  PiecewiseQuadratic1d substitute(const PiecewiseLinear& conditional) const;

  /// Applies `func` to each of the segments where we have a distinct region
  /// between the quadratic (piecewise over x) and conditional (linear piecewise
  /// over y) regions.
  /// Iteration is guaranteed to occur in ascending order of y.
  /// @param func(x_segment_index, y_segment_index) is called for each segment,
  /// where x_segment `i` is bounded to-the-right by xc[i] and y_segment `j` is
  /// bounded above by yc[j]
  static void iterateOverXcYcSegments(
      const Vec& xc, const PiecewiseLinear& conditional,
      const std::function<void(int x_segment_index, int y_segment_index,
                               double y_upper_bound)>& func);

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
    std::cout << s << "C:\n" << C_ << "\nxc:\n" << xc_ << std::endl;
  }
  bool equals(const This& other, double tol = 1e-9) const {
    return traits<Mat>::Equals(C_, other.C_, tol) &&
           traits<Vec>::Equals(xc_, other.xc_, tol);
  }

  // Getters
  const auto a() const { return C_.col(0); }
  const auto b() const { return C_.col(1); }
  const auto c() const { return C_.col(2); }
  const auto d() const { return C_.col(3); }
  const auto e() const { return C_.col(4); }
  const auto f() const { return C_.col(5); }
  const Vec& xc() const { return xc_; }
  const Mat& C() const { return C_; }
  auto a() { return C_.col(0); }
  auto b() { return C_.col(1); }
  auto c() { return C_.col(2); }
  auto d() { return C_.col(3); }
  auto e() { return C_.col(4); }
  auto f() { return C_.col(5); }
  Vec& xc() { return xc_; }
  Mat& C() { return C_; }

 private:
  // Member variables
  Mat C_;
  Vec xc_;
};

template <>
struct traits<PiecewiseQuadratic> : public Testable<PiecewiseQuadratic> {};
template <>
struct traits<PiecewiseQuadratic1d> : public Testable<PiecewiseQuadratic1d> {};

}  // namespace gtsam
