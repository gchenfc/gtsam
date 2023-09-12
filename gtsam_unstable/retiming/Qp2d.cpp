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
  // Returns false if infeasible
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
  // Start with the last one because it has a for-loop issue anyway
  PiecewiseQuadratic1d sol;
  double last_x_bound = (objective.xc().size() == 0)
                            ? -std::numeric_limits<double>::infinity()
                            : objective.xc().tail<1>().value();
  if (!insert_x_lims(last_x_bound, std::numeric_limits<double>::infinity())) {
    double inf = std::numeric_limits<double>::infinity();
    // Infinite cost, feasible nowhere
    sol = PiecewiseQuadratic1d{.C = Eigen::Matrix<double, 1, 3>{0, 0, inf},
                               .xc = Eigen::Vector2d{0, 0}};
  } else {
    sol = min(objective.C().bottomRows<1>(), ineqs_with_x_limits_sorted);
  }

  // Now loop through every other quadratic segment
  double x_min = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < objective.xc().size(); ++i) {
    double x_max = objective.xc()(i);
    if (x_min == x_max) continue;
    insert_x_lims(x_min, x_max);
    const auto& sol2 = min(objective.C().row(i), ineqs_with_x_limits_sorted);
    PiecewiseQuadratic1d::MinInPlace(sol, sol2);
    x_min = x_max;
  }

  // Finally, we need to switch from the convention of including limits in xc to
  // without
  if (bounds_on_argument) {
    *bounds_on_argument << 1, sol.xc(sol.xc.size() - 1),  //
        -1, -sol.xc(0);
  }
  Eigen::VectorXd xc_tmp = sol.xc.segment(1, sol.xc.size() - 2);
  sol.xc = xc_tmp;

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
    auto next_vertex =
        lp2d::nextVertexFromSorted(inequalities, i, ccw == to_max)(1);
    printf("next vertex: %f, last vertex: %f\n", next_vertex, xc.back());
    if (next_vertex == xc.back()) continue;
    qs.emplace_back(::gtsam::internal::substitute(objective, m_, b_));
    xc.emplace_back(next_vertex);
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

  // Check if the last 2 are redundant
  if ((xc.size() >= 2) && (xc.back() == xc.at(xc.size() - 2))) {
    xc.pop_back();
    qs.pop_back();
  }
}

/******************************************************************************/

/// @brief Computes the gradient of the objective at a given point
std::pair<double, double> gradient(const Eigen::Ref<const Quadratic>& objective,
                                   double x, double y) {
  const auto &a = objective(0), &b = objective(1), &c = objective(2),
             &d = objective(3), &e = objective(4);  // &f = objective(5);
  return {2 * a * x + c * y + d, 2 * b * y + c * x + e};
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
  //    Unconstrained sol of the form ~~~x = m.y + b~~~
  // x = -(c.y + d) / (2a)
  // 2a.x + c.y = -d
  auto xys = lp2d::computeAllIntersections(inequalities, {2 * a, c, -d});
  const auto& ys = xys.col(1).array();

  const auto [lower_intersection, upper_intersection] =
      lp2d::computeFeasiblePointPair(inequalities, xys);

  printf("The active inequalities are %d %d\n", lower_intersection,
         upper_intersection);

  if ((lower_intersection == -1) && (upper_intersection == -1)) {
    // Could either be unbounded or doesn't intersect with the polygon at all
    if (lp2d::isFeasible(inequalities, lp2d::Point(b, 0))) {  // unbounded
      return PiecewiseQuadratic1d{
          .C = ::gtsam::internal::substitute(objective, m, b),
          .xc = Eigen::Vector2d{-std::numeric_limits<double>::infinity(),
                                std::numeric_limits<double>::infinity()}};
    } else {  // doesn't intersect with polygon - follow the gradient direction
      // Find lowest edge
      // Note: min_edge is on the left side of the vertex
      int min_edge = lp2d::traverseSortedToExtremal(inequalities, 0, true);
      if ((inequalities(min_edge, 1) > 0) || (inequalities(min_edge, 0) > 0)) {
        // we are at a max, traverse again to get to min
        min_edge = (min_edge + 1) % inequalities.rows();
        min_edge = lp2d::traverseSortedToExtremal(inequalities, min_edge, true);
      }
      auto min_vertex =
          lp2d::nextVertexFromSorted(inequalities, min_edge, true);
      if (gradient(objective, min_vertex(0), min_vertex(1)).first > 0) {
        // gradient right -> descent left
        computeObjectiveAlongBoundary(inequalities, objective, min_edge,
                                      /*to_max*/ true,
                                      min_vertex(1),  //
                                      qs, xc);
      } else {  // gradient left -> descent right
        // advance min_edge by 1-stop ccw
        min_edge = (min_edge + 1) % inequalities.rows();
        computeObjectiveAlongBoundary(inequalities, objective, min_edge,
                                      /*to_max*/ true,
                                      min_vertex(1),  //
                                      qs, xc);
      }
      auto ret = PiecewiseQuadratic1d::Create(qs, xc);
      ret.print("Detected lack of intersection.  Returning: ");
      return ret;
    }
  }

  // First traverse until the lower bound
  if (lower_intersection == -1) {
    // No lower bound, so we start at -infinity
    xc.emplace_back(-std::numeric_limits<double>::infinity());
  } else {
    // Traverse from min to intersection
    computeObjectiveAlongBoundary(inequalities, objective, lower_intersection,
                                  /*to_max*/ false,
                                  ys(lower_intersection),  //
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

  return PiecewiseQuadratic1d::Create(qs, xc);
}

}  // namespace qp2d
}  // namespace gtsam
