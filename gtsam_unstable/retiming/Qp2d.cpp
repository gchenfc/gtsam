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

  Inequalities inequalities_sorted;
  if (!lp2d::sortBoundaries(inequalities, inequalities_sorted)) {
    throw std::runtime_error("Infeasible");
  }
  Inequalities x_bounds(2, 3);
  Inequalities ineqs_with_x_limits_sorted;
  auto insert_x_lims = [&inequalities_sorted, &x_bounds,
                        &ineqs_with_x_limits_sorted](double x_min,
                                                     double x_max) -> bool {
    x_bounds << 1, 0, x_max,  //
        -1, 0, -x_min;
    return lp2d::insertBoundariesSorted(inequalities_sorted, x_bounds,
                                        ineqs_with_x_limits_sorted);
  };

  // We build-it out by starting with some solution then merging in one at a
  // time
  // Start with the last one because it has a for loop issue anyway
  PiecewiseQuadratic1d sol;
  double last_x_bound = (objective.xc().size() == 0)
                            ? -std::numeric_limits<double>::infinity()
                            : objective.xc().tail<1>().value();
  if (!insert_x_lims(last_x_bound, std::numeric_limits<double>::infinity())) {
    double inf = std::numeric_limits<double>::infinity();
    sol = PiecewiseQuadratic1d{.C = Eigen::Matrix<double, 1, 3>{0, 0, inf},
                               .xc = Eigen::VectorXd(0)};
  } else {
    sol = min(objective.C().bottomRows<1>(), ineqs_with_x_limits_sorted);
  }

  // Now loop through every other quadratic segment
  double x_min = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < objective.xc().size(); ++i) {
    double x_max = objective.xc()(i);
    insert_x_lims(x_min, x_max);
    const auto& sol2 = min(objective.C().row(i), ineqs_with_x_limits_sorted);
    PiecewiseQuadratic1d::MinInPlace(sol, sol2);
    x_min = x_max;
  }

  return sol;
}

/******************************************************************************/

/// @brief Evaluates the objective around the boundary starting from a given
/// index and going till the minimum/maximum vertex.  Automatically sorts to go
/// from min to max.
/// @param inequalities The (sorted!) inequalities defining the boundary
/// @param objective The objective function in the form [a, b, c, d, e, f]
/// @param intersected_edge_index The index of the edge that the unconstrained
/// solution to the objective intersects
/// @param to_max Whether we want to traverse to the max or to the min
/// @param[out] qs The objective function coeffs (evaluated along the boundary)
/// @param[out] xc The segment boundaries, with size qs.rows() + 1
void computeObjectiveAlongBoundary(const Inequalities& inequalities,
                                   const Eigen::Ref<const Quadratic>& objective,
                                   int intersected_edge_index, bool to_max,
                                   double y_at_intersection,
                                   // output args:
                                   std::vector<Eigen::Matrix<double, 1, 3>>& qs,
                                   std::vector<double>& xc) {
  const auto &A = inequalities.col(0), &B = inequalities.col(1),
             &C = inequalities.col(2);

  bool ccw = (A(intersected_edge_index) > 0) == to_max;

  // Find the final edge / vertex
  int extremal_edge_index =
      lp2d::traverseSortedToExtremal(inequalities, intersected_edge_index, ccw);

  // Start and End edges
  auto start_edge = to_max ? intersected_edge_index : extremal_edge_index;
  auto end_edge = to_max ? extremal_edge_index : intersected_edge_index;

  // Add start vertex
  if (to_max) {
    xc.emplace_back(y_at_intersection);
  } else {
    xc.emplace_back(
        lp2d::nextVertexFromSorted(inequalities, start_edge, ccw != to_max)(1));
  }

  // Add middle objectives & vertices
  for (int i = start_edge; i != end_edge;
       i = (i + (ccw ? 1 : inequalities.rows() - 1)) % inequalities.rows()) {
    // inequality looks like ax + by <= c, but we need in the form
    //                        x = m.y + b
    if (std::abs(A(i)) < 1e-9) continue;  // horizontal line
    double m_ = -B(i) / A(i), b_ = C(i) / A(i);
    qs.emplace_back(::gtsam::internal::substitute(objective, m_, b_));
    xc.emplace_back(
        lp2d::nextVertexFromSorted(inequalities, i, ccw == to_max)(1));
  }
  // Add objective for final
  double m_ = -B(end_edge) / A(end_edge), b_ = C(end_edge) / A(end_edge);
  qs.emplace_back(::gtsam::internal::substitute(objective, m_, b_));

  // Add end vertex
  if (to_max) {
    xc.emplace_back(
        lp2d::nextVertexFromSorted(inequalities, end_edge, ccw == to_max)(1));
  } else {
    xc.emplace_back(y_at_intersection);
  }
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
  double m = -c / (2 * a);
  double b = -d / (2 * a);

  // Now find where the unconstrained solution intersects the inequalities
  //    Inequalities of the form Ax + By <= C
  //    Unconstrained sol of the form x = m.y + b
  const auto &A = inequalities.col(0).array(), &B = inequalities.col(1).array(),
             &C = inequalities.col(2).array();
  const auto& ys = (C - A * b) / (A * m + B);
  const auto& xs = m * ys + b;
  Eigen::ArrayXXd lhs = (A.matrix() * xs.matrix().transpose() +
                         B.matrix() * ys.matrix().transpose())
                            .array();
  Eigen::ArrayXd rhs = (C + 1e-9).matrix();
  const auto& is_feasibles = ((lhs.colwise() - rhs) <= 0).colwise().all();
  // Separate out lower vs upper intersection
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
  printf("lower_intersection = %d, upper_intersection = %d\n",
         lower_intersection, upper_intersection);

  // First traverse until the lower bound
  if (lower_intersection == -1) {
    // No lower bound, so we start at -infinity
    xc.emplace_back(-std::numeric_limits<double>::infinity());
  } else {
    // Traverse from min to intersection
    computeObjectiveAlongBoundary(inequalities, objective, lower_intersection,
                                  /*to_max*/ false, ys(lower_intersection),  //
                                  qs, xc);
  }

  // Middle is the unconstrained solution
  qs.emplace_back(::gtsam::internal::substitute(objective, m, b));

  // Finally, traverse until the upper bound
  if (upper_intersection == -1) {  // unbounded
    xc.emplace_back(std::numeric_limits<double>::infinity());
  } else {
    // traverse from intersection to max
    computeObjectiveAlongBoundary(inequalities, objective, upper_intersection,
                                  /*to_max*/ true, ys(upper_intersection),  //
                                  qs, xc);
  }

  // Export
  PiecewiseQuadratic1d sol;
  sol.C.resize(qs.size(), 3);
  sol.xc.resize(xc.size());
  std::copy(qs.begin(), qs.end(), sol.C.rowwise().begin());
  std::copy(xc.begin(), xc.end(), sol.xc.data());
  return sol;
}

}  // namespace qp2d
}  // namespace gtsam
