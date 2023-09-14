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

  /// @brief Construct from std::vectors
  static PiecewiseQuadratic1d Create(
      const std::vector<Eigen::Matrix<double, 1, 3>>& Cs,
      const std::vector<double>& xcs) {
    PiecewiseQuadratic1d result{decltype(C)(Cs.size(), 3),
                                decltype(xc)(xcs.size())};
    std::copy(Cs.begin(), Cs.end(), result.C.rowwise().begin());
    std::copy(xcs.begin(), xcs.end(), result.xc.data());
    return result;
  }

  /// @brief Takes the min of a pair of piecewise quadratics
  static void MinInPlace(PiecewiseQuadratic1d& q1,
                         const PiecewiseQuadratic1d& q2);

  /// @brief Eliminates unnecessary xc's by merging identical adjacent segments
  static void SmoothenInPlace(PiecewiseQuadratic1d& q);

  /// @brief Eliminates unnecessary xc's by merging identical adjacent segments
  static void SmoothenInPlace(std::vector<Eigen::Matrix<double, 1, 3>>& C,
                              std::vector<double>& xc);

  /// @brief Evaluate the piecewise quadratic at a point x
  double evaluate(double x) const {
    auto i =
        std::distance(xc.begin(), std::upper_bound(xc.begin(), xc.end(), x));
    if (xc.size() > C.rows()) --i;  // compensate for bookends
    if (i < 0) throw std::runtime_error("x is out of bounds");
    return C(i, 0) * x * x + C(i, 1) * x + C(i, 2);
  }

  /// @brief Evaluate the global minimum location x
  double argmin() const;

  // Testable
  void print(const std::string& s = "Piecewise Quadratic 1d",
             const KeyFormatter& formatter = DefaultKeyFormatter) const {
    std::cout << s << "C:\n"
              << WithIndent(C, "\t\t")  //
              << "\txc: " << xc.transpose() << std::endl;
  }
  bool equals(const PiecewiseQuadratic1d& other, double tol = 1e-9) const {
    return traits<decltype(C)>::Equals(C, other.C, tol) &&
           traits<decltype(xc)>::Equals(xc, other.xc, tol);
  }
};

namespace internal {
/// @brief Substitute a linear equality (x = m.y + b) into a quadratic to obtain
/// a quadratic in y
Eigen::Matrix<double, 1, 3> substitute(
    const Eigen::Ref<const Eigen::Matrix<double, 1, 6>>& q,  //
    const double m, const double b);
}  // namespace internal

/**
 * @brief A piecewise quadratic used to define objectives.
 *
 * The following representation is used for a quadratic:
 *    f(x) = a.x^2 + b.y^2 + c.x.y + d.x + e.y + f
 *    NOT THIS: f(x) = [x'; y']^T * Q * [x'; y']
 *              where Q is a 2x2 matrix.
 * This is convenient because:
 *  - we can represent the "piecewise" part with *vectors* a, b, c, d, e, f
 * to vectorize calculations
 *  - parametric solution is easy: x'* = -(c.y + d) / (2a)
 *
 * A piecewise quadratic will be represented with vectors a, b, c, d, e, f,
 * and a vector of dividing points x.  Note that for our application it will
 * always be piecewise in x only.
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
  /// piecewise linear solution f(x^*(y), y) and inequality bounds on the
  /// argument y.
  std::pair<PiecewiseQuadratic, Bounds1d> solveParametric(
      const Inequalities& inequalities) const;

  /// Substitute a piecewise linear solution x^*(y) into the quadratic to obtain
  /// a new piecewise quadratic on y (one variable, so b, c, e = 0).
  /// We might not even need this entire function because it's more efficient
  /// for solveParametric to return the objective function rather than the
  /// conditional, but it's good practice I guess
  PiecewiseQuadratic1d substitute(const PiecewiseLinear& conditional) const;

  /// Substitute a linear solution x^*(y) into the quadratic to obtain
  /// a new piecewise quadratic on y (one variable, so b, c, e = 0).
  /// conditional should take the form ax + by = c
  PiecewiseQuadratic1d substituteEq1(
      const LinearConstraint::Linear& conditional) const;

  /// Substitute a linear solution x^*(y, z) into the quadratic to obtain
  /// a new piecewise quadratic on y and z
  /// conditional should take the form ax + by + cz = d
  PiecewiseQuadratic substituteEq2(
      const LinearConstraint::Linear& conditional) const;

  /// Substitute a linear solution into the quadratic to obtain a new piecewise
  /// quadratic on the separator
  PiecewiseQuadratic substitute(
      const LinearConstraint::Linear& conditional) const {
    if (conditional.cols() == 3) return substituteEq1(conditional);
    if (conditional.cols() == 4) return substituteEq2(conditional);
    throw std::runtime_error("Separator should be 1 or 2 variables");
  }

  /// Substitute a value for y into the quadratic to obtain a new piecewise
  /// quadratic on y (one variable, so b, c, e = 0)
  PiecewiseQuadratic1d substitute(double y) const;

  /// The opposite of the constructor from PiecewiseQuadratic1d.  If the columns
  /// are 0, then we can convert to PiecewiseQuadratic1d.
  PiecewiseQuadratic1d as1d(bool check_columns = true) const;

  /// @brief Solve argmin, given y
  double argmin(double y) const { return substitute(y).argmin(); }

  /// @brief Eliminates unnecessary xc's by merging identical adjacent segments
  static void SmoothenInPlace(std::vector<Eigen::Matrix<double, 1, 6>>& C,
                              std::vector<double>& xc);

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

  /// Swap the x/y variables
  static PiecewiseQuadratic swapXy(const PiecewiseQuadratic& src);

  /// Potentially swap the columns of C_ if we want to reverse the key order
  PiecewiseQuadratic rekey(const KeyVector& src_keys,
                           const KeyVector& dest_keys) const;

  /// Returns true if the piecewise quadratic is useless (all zeros)
  bool empty() const {
    return (C_.array() == 0).all() && (xc_.array() == 0).all();
  }

  // Testable
  void print(const std::string& s = "Piecewise Quadratic",
             const KeyFormatter& formatter = DefaultKeyFormatter) const {
    std::cout << s << "C:\n"
              << WithIndent(C_, "\t\t")  //
              << "\txc: " << xc_.transpose() << std::endl;
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
