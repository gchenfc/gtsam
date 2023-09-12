#include "Qp2d.h"

#include "Lp2d.h"

namespace gtsam {
namespace qp2d {

PiecewiseQuadratic1d min(const PiecewiseQuadratic& objective,
                         const Inequalities& inequalities,
                         Bounds1d* bounds_on_argument) {
  /* We will need to split this up into 2 nested loops:
   *  - Each x-segment defining a separate quadratic
   *  - Each y-segment defining a separate pair of linear inequalities
   * I don't think the inner/outer order makes a difference.
   * For coding readibility, I will do the outer loop over x-segments then merge
   * at the end, so pseudocode:
   *
   * func min(qs, inequalities) -> PiecewiseQuadratic1d:
   *    PiecewiseQuadratic1d sols = []
   *    for each x-segment:
   *        sols.append (min(q_xseg, inequalities & xmin <= x <= xmax))
   *    return merge(sols)
   *
   * func merge is "elementwise min"
   *
   * func min(q, inequalities) -> PiecewiseQuadratic1d:
   *    compute un-constrained solution
   *    bound it by constrained solution
   *    evaluate q(x, y) along solution
   */

  Inequalities inequalities_with_x_limits(inequalities.rows() + 2, 3);
  inequalities_with_x_limits.topRows(inequalities.rows()) = inequalities;

  double last_x_bound = (objective.xc().size() == 0)
                            ? -std::numeric_limits<double>::infinity()
                            : objective.xc().tail<1>().value();
  inequalities_with_x_limits.bottomRows<2>() << 1, 0,
      std::numeric_limits<double>::infinity(), -1, 0, -last_x_bound;
  PiecewiseQuadratic1d sol = min(objective.C().bottomRows<1>(), inequalities);

  double x_min = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < objective.xc().size(); ++i) {
    double x_max = objective.xc()(i);
    inequalities_with_x_limits.bottomRows<2>() << 1, 0, x_max, -1, 0, -x_min;
    const auto& sol2 = min(objective.C().row(i), inequalities_with_x_limits);
    PiecewiseQuadratic1d::MinInPlace(sol, sol2);
    x_min = x_max;
  }

  return sol;
}

/******************************************************************************/

PiecewiseQuadratic1d min(const Eigen::Ref<const Quadratic>& objective,
                         const Inequalities& inequalities) {
  // Solution to return
  std::vector<Eigen::Matrix<double, 1, 3>> qs;
  std::vector<double> xc;

  // First compute the unconstrained solution
  //  x'* = -(c.y + d) / (2a)
  //  x'* = (-c/(2a)) * y + (-d / (2a)) = m.y + b
  const auto &a = objective(0), &c = objective(2), &d = objective(3);
  if (std::abs(a) < 1e-9) {
    throw std::runtime_error("TODO: switch to LP");
  }
  double m = -c / (2 * a), b = -d / (2 * a);

  // Now find where the unconstrained solution intersects the inequalities
  // Inequalities of the form Ax + By <= C
  const auto &A = inequalities.col(0).array(), &B = inequalities.col(1).array(),
             &C = inequalities.col(2).array();
  const auto& ys = (C - A * b) / (A * m + B);
  const auto& xs = m * ys + b;
  const auto& is_feasibles = (A * xs + B * ys) <= (C + 1e-9);
  int lower_intersection = -1, upper_intersection = -1;
  for (int i = 0; i < is_feasibles.size(); ++i) {
    if (is_feasibles(i)) {
      if (B(i) > 0) {  // upper bound
        assertm(upper_intersection == -1, "Multiple upper intersections");
        upper_intersection = i;
      } else {
        assertm(lower_intersection == -1, "Multiple lower intersections");
        lower_intersection = i;
      }
    }
  }

  // First traverse until the lower bound
  if (lower_intersection == -1) {
    // No lower bound, so we start at -infinity
    xc.emplace_back(-std::numeric_limits<double>::infinity());
  } else {
    // find the final edge (min edge)
    bool ccw = A(lower_intersection) < 0;
    int min_edge =
        lp2d::traverseSortedToExtremal(inequalities, lower_intersection, ccw);
    if (ccw) {
      // First add the ccw vertex to edge
      for (int i = min_edge; i != lower_intersection;
           i = (i + 1) % inequalities.rows()) {
        // inequality looks like ax + by <= c, but we need in the form
        //                        x = m.y + b
        if (std::abs(A(i)) < 1e-9) continue;  // horizontal line
        double m = -B(i) / A(i), b = C(i) / A(i);
        qs.emplace_back(::gtsam::internal::substitute(objective, m, b));
        xc.emplace_back(xs(i));
      }
    }
    // qs.emplace_back(objective);
    // xc.emplace_back(xs(lower_intersection));
  }

  // Middle is the unconstrained solution

  // End is the

  return PiecewiseQuadratic1d{};
}

}  // namespace qp2d
}  // namespace gtsam
