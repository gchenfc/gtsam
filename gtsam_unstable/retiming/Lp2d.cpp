#include "Lp2d.h"

#include <iostream>

#include "assert.h"

#ifdef USE_TOPPRA_LP2D
// I tried theirs, and it's no faster
// Add these lines to gtsam_unstable/CMakeLists.txt:
//  find_package(toppra)
//  target_link_libraries(gtsam_unstable PUBLIC toppra::toppra)
#include <toppra/solver/seidel-internal.hpp>
using namespace toppra::solver;
#endif // USE_TOPPRA_LP2D

namespace gtsam {
namespace lp2d {

/******************************************************************************/

#ifdef USE_TOPPRA_LP2D
ScalarBounds extremalsY_(const Inequalities& inequalities) {
  Seidel a;

  MatrixX2 A_1d(inequalities.rows() + 4, 2);
  Inequalities A(inequalities.rows(), 3);
  A << inequalities.col(0), inequalities.col(1), -inequalities.col(2);

  seidel::LpSol sol = seidel::solve_lp2d(
      RowVector2{0, 1}, A, Vector2{-999, -999}, Vector2{999, 999}, A_1d);
  assertm(sol.feasible, "Infeasible");
  double max = sol.optvar(1);

  sol = seidel::solve_lp2d(RowVector2{0, -1}, A, Vector2{-999, -999},
                           Vector2{999, 999}, A_1d);
  assertm(sol.feasible, "Infeasible");
  double min = sol.optvar(1);

  ScalarBounds bounds;
  bounds << 1, max, -1, -min;
  return bounds;
}
#endif // USE_TOPPRA_LP2D

ScalarBounds extremalsY(const Inequalities& inequalities) {
  // TODO(gerry): Here's a simplex-like algorithm that might be more efficient:
  //    When we determine a vertex in infeasible, instead of just checking the
  //    next vertex (exhaustivly), instead the next vertex to try should be
  //    the intersection of (a line) with (the inequality/line that caused the
  //    infeasibility check to fail).
  //    "a line" may have to generate a vertex to try for both lines, and take
  //    either which satisfies all 3 constraints (or all the constraints we've
  //    seen so far?).
  //    Repeat until we get a feasible point.
  //    Once we get a feasible point, traverse upwards (or downwards) until we
  //    find the extremal.
  // I suspect this is not more efficient because all the extra book-keeping &
  // branch mis-prediction isn't worth it for the small number of inequalities
  // we have.
  ScalarBounds bounds = ScalarBounds::Zero();

  // Loop over every unique pair of inequalities
  for (int i = 0; i < inequalities.rows(); ++i) {
    for (int j = i + 1; j < inequalities.rows(); ++j) {
      const auto& l1 = inequalities.row(i);
      const auto& l2 = inequalities.row(j);

      // We only care about bottom-boundaries or top-boundaries
      // One way to check this is to check if the signs of the x-components of
      // the normal vectors are different.  Special case: if either has cx=0,
      // that's a horizontal line and we want to keep it.
      if (l1(0) * l2(0) > 0) continue;

      // Manually check for parallel lines to reduce catch/throw overhead
      if (abs(l1(0) * l2(1) - l1(1) * l2(0)) < 1e-12) continue;

      // Compute intersection point
      auto p = intersection(l1, l2);
      if (!isFeasible(inequalities, p)) continue;

      // To determine whether it's an upper or lower bound, check the
      // direction of the "averaged" normal vector is pointing up or down.
      // Note: due to the <= convention, normal vector points away from the
      // feasible region
      // Also, because it's convex, this is the global max
      if ((l1.head<2>().normalized() + l2.head<2>().normalized())(1) < 0) {
        // vector pointing down -> feasible region above -> lower bound
        bounds.row(1) << -1, -p(1);       // -y <= -p1 (lower bound)
        if (bounds(0, 0)) return bounds;  // early exit
      } else {
        bounds.row(0) << 1, p(1);         // y <= p1 (upper bound)
        if (bounds(1, 0)) return bounds;  // early exit
      }
    }
  }

  return bounds;
}

/******************************************************************************/

int findCcwIntersection(const Eigen::Ref<const Inequalities>& inequalities,
                        int start_index, bool ccw = true) {
  // Find the first inequality that is ccw with the point
  for (int i = 0; i < inequalities.rows(); ++i) {
    if (i == start_index) continue;
    Point p = intersection(inequalities.row(start_index), inequalities.row(i));
    if (isFeasible(inequalities, p) &&
        (isCcw(inequalities.row(start_index), inequalities.row(i)) == ccw)) {
      return i;
    }
  }
  return -1;
}

// This is a special case of sortBoundaries when there are no intersections
bool sortInequalitiesNoIntersections(const Inequalities& inequalities,
                                     Inequalities& result) {
  // This can happen if either there are no feasible intersections, or there
  // are no intersections to begin with (all are parallel)
  // Let's check if they're all parallel
  for (int i = 1; i < inequalities.rows(); ++i) {
    if (abs(inequalities(i, 0) * inequalities(0, 1) -
            inequalities(i, 1) * inequalities(0, 0)) > 1e-12) {
      // these are not parallel, so must be infeasible
      return false;  // infeasible
    }
  }
  // Now we know they are all parallel, but they could be pointing outwards,
  // e.g. x > 1, and x < -1
  // For each line, find a random point on it and check if it's infeasible
  result = Inequalities(2, 3);
  int num_feasible = 0;
  if (std::abs(inequalities(0, 0)) > 1e-8) {  // try y = 0
    for (int i = 0; i < inequalities.rows(); ++i) {
      if (isFeasible(inequalities,
                     Point(inequalities(i, 2) / inequalities(i, 0), 0))) {
        result.row(num_feasible++) = inequalities.row(i);
      }
    }
  } else {  // do x = 0
    for (int i = 0; i < inequalities.rows(); ++i) {
      if (isFeasible(inequalities,
                     Point(0, inequalities(i, 2) / inequalities(i, 1)))) {
        result.row(num_feasible++) = inequalities.row(i);
      }
    }
  }
  if (num_feasible == 0) return false;
  result.conservativeResize(num_feasible, Eigen::NoChange);
  return true;
}

void sortInequalitiesCw(const Inequalities& inequalities, int start_index,
                        int end_index, Inequalities& output) {
  // Search clockwise starting from start_index, and stop if we ever hit
  // end_index.  Responsibility is on caller to allocate output
  int cur_index = start_index;
  for (int out_index = 0; out_index < output.rows(); ++out_index) {
    int next_index = findCcwIntersection(inequalities, cur_index, false);
    if (next_index == end_index) {
      output.conservativeResize(out_index, Eigen::NoChange);
      return;
    } else if (next_index == -1) {
      // Reached an open boundary, return
      output.conservativeResize(out_index, Eigen::NoChange);
      return;
    } else {
      output.row(out_index) = inequalities.row(next_index);
      cur_index = next_index;
    }
  }
  int sanity_check_index = findCcwIntersection(inequalities, cur_index, false);
  if ((sanity_check_index != end_index) && (sanity_check_index != -1)) {
    throw std::runtime_error("Not enough space allocated");
  }
  // No resizing needed
}

bool sortBoundaries(const Inequalities& inequalities, Inequalities& result) {
  if (inequalities.rows() <= 2) {
    result = inequalities;
    // If the 2 inequalities are negative of eachother, then the feasible region
    // is empty (a single line)
    if (((result.row(0) + result.row(1)).array().abs() < 1e-9).all()) {
      return false;
    }
    return true;
  }

  result.resize(inequalities.rows(), 3);
  // Handle first pair of edges separately
  int start_index = 0, most_recent_added;
  for (; start_index < inequalities.rows(); ++start_index) {
    most_recent_added = findCcwIntersection(inequalities, start_index);
    if (most_recent_added != -1) {
      result.row(0) = inequalities.row(start_index);
      result.row(1) = inequalities.row(most_recent_added);
      break;
    }
  }
  if (start_index == inequalities.rows()) {
    return sortInequalitiesNoIntersections(inequalities, result);
  }

  // Now loop over the rest of the edges
  for (int index = 1; index < inequalities.rows(); ++index) {
    int to_add = findCcwIntersection(inequalities, most_recent_added);
    if (to_add == -1) {
      // The feasible region is open, so we need to go cw starting from
      // start_index
      Inequalities cw(inequalities.rows() - (index + 1), 3);
      sortInequalitiesCw(inequalities, start_index, most_recent_added, cw);
      // Insert cw backwards from the end
      for (int k = cw.rows() - 1; k >= 0; --k) {
        result.row(index + cw.rows() - k) = cw.row(k);
      }
      result.conservativeResize(index + 1 + cw.rows(), Eigen::NoChange);
      return true;
    } else if (to_add == start_index) {
      // We've looped back to the start, so we're done
      result.conservativeResize(index + 1, Eigen::NoChange);
      return true;
    } else {
      // Insert the new inequality and move on
      result.row(index + 1) = inequalities.row(to_add);
    }
    most_recent_added = to_add;
  }

  return false;  // infeasible
}

/******************************************************************************/

/// @brief Insert new inequalities while maintaining sorted property
bool insertBoundariesSorted(const Inequalities& inequalities,
                            const Inequalities& new_inequalities,
                            Inequalities& result) {
  if (new_inequalities.rows() == 0) {
    result = inequalities;
    return true;
  }
  // TODO(gerry): implement this more efficiently
  Inequalities tmp(inequalities.rows() + new_inequalities.rows(), 3);
  tmp << inequalities, new_inequalities;
  return sortBoundaries(tmp, result);
}

/******************************************************************************/

int traverseSortedToExtremal(const Inequalities& inequalities, int start_index,
                             bool ccw) {
  if (inequalities.rows() == 1) return 0;

  const auto& outside_dir = inequalities.col(0);
  if (outside_dir(start_index) == 0) return start_index;
  bool is_left = outside_dir(start_index) < 0;
  auto is_connected_edge = [&inequalities, &ccw](int this_edge, int next_edge) {
    // returns false if the two edges are connected due to unboundedness
    return isCcw(inequalities.row(this_edge), inequalities.row(next_edge)) ==
           ccw;
  };

  int min_edge;
  int next_edge = start_index;
  do {
    min_edge = next_edge;
    next_edge = (ccw ? (min_edge + 1) : (min_edge + inequalities.rows() - 1)) %
                inequalities.rows();
  } while (((outside_dir(next_edge) < 0) == is_left) &&
           (outside_dir(next_edge) != 0) &&
           (is_connected_edge(min_edge, next_edge)));

  return min_edge;
}

/******************************************************************************/

Point nextVertexFromSorted(const Inequalities& inequalities, int edge_index,
                           bool ccw) {
  int next_edge =
      (edge_index + (ccw ? 1 : inequalities.rows() - 1)) % inequalities.rows();
  if (isCcw(inequalities.row(edge_index), inequalities.row(next_edge)) == ccw) {
    return intersection(inequalities.row(edge_index),
                        inequalities.row(next_edge));
  } else {
    // unbounded
    double x_coeff = inequalities(edge_index, 0);
    double y_coeff = inequalities(edge_index, 1);
    double x = ((ccw == (y_coeff < 0)) ? 1 : -1) *
               std::numeric_limits<double>::infinity();
    double y = ((ccw == (x_coeff < 0)) ? -1 : 1) *
               std::numeric_limits<double>::infinity();
    return Point(x, y);
  }
}

/******************************************************************************/

Eigen::Array<double, Eigen::Dynamic, 2> computeAllIntersections(
    const Inequalities& inequalities, const Inequality& line) {
  // Vectorized version of `intersection` below
  const auto &A = inequalities.col(0), &B = inequalities.col(1),
             &E = inequalities.col(2);
  const auto &c = line(0), &d = line(1), &e = line(2);

  const auto& inv_determinant = 1 / (A * d - B * c).array();
  Eigen::Array<double, Eigen::Dynamic, 2> intersections(inequalities.rows(), 2);
  intersections.col(0) = (d * E - B * e).array() * inv_determinant;
  intersections.col(1) = (-c * E + A * e).array() * inv_determinant;
  return intersections;
}

/******************************************************************************/

/// @brief Computes the indices of the feasible points
std::pair<int, int> computeFeasiblePointPair(
    const Inequalities& inequalities,
    const Eigen::Array<double, Eigen::Dynamic, 2>& intersections,
    const Inequality& line, double tol) {
  const auto &A = inequalities.col(0).array(), &B = inequalities.col(1).array(),
             &C = inequalities.col(2).array();
  const auto& xs = intersections.col(0);
  const auto& ys = intersections.col(1);

  Eigen::ArrayXXd lhs = (A.matrix() * xs.matrix().transpose() +
                         B.matrix() * ys.matrix().transpose())
                            .array();
  Eigen::ArrayXd rhs = (C + tol).matrix();
  const auto& is_feasibles = ((lhs.colwise() - rhs) <= 0).colwise().all();

  double xt = line(1), yt = -line(0);
  auto is_upper = [&xt, &yt](const auto& inequality) {
    if (std::abs(inequality(0)) < 1e-9) return (inequality(1) > 0);
    double line_dot_ineq = xt * inequality(0) + yt * inequality(1);
    // if they are in the same direction and yt is positive, then upper bound
    return (line_dot_ineq > 0) == (yt > 0);
  };

  // Separate out lower vs upper intersection
  int lower_intersection = -1, upper_intersection = -1;
  for (int i = 0; i < is_feasibles.size(); ++i) {
    if (is_feasibles(i)) {
      if (is_upper(inequalities.row(i))) {  // upper bound
        if (upper_intersection != -1) {
          if ((std::abs(xs(i) - xs(upper_intersection)) > 1e-9) ||
              (std::abs(ys(i) - ys(upper_intersection)) > 1e-9)) {
            assertm(false, "Multiple distinct upper intersections");
          }
        }
        upper_intersection = i;
      } else {
        if (lower_intersection != -1) {
          if ((std::abs(xs(i) - xs(lower_intersection)) > 1e-9) ||
              (std::abs(ys(i) - ys(lower_intersection)) > 1e-9)) {
            assertm(false, "Multiple distinct lower intersections");
          }
        }
        lower_intersection = i;
      }
    }
  }

  return {lower_intersection, upper_intersection};
}

/******************************************************************************/

bool isCcw(const Inequality& line1, const Inequality& line2) {
  // Check if the inequalities are going ccw or cw.  Return true for ccw
  return (line1(0) * line2(1) - line1(1) * line2(0)) > 0;
};

/******************************************************************************/

Point intersection(const Inequality& line1, const Inequality& line2,
                   double parallel_tol) {
  // [a, b] . [x] = [e1]
  // [c, d]   [y] = [e2]
  // xy = Ainv * e = (1/(ad-bc)) * [d, -b; -c, a] * [e1; e2]

  const double &a = line1(0), &b = line1(1), &e1 = line1(2),
               /**/ &c = line2(0), &d = line2(1), &e2 = line2(2);

  double det = a * d - b * c;
  // assertm(abs(det) >= parallel_tol, "Lines are parallel");
  if (abs(det) < parallel_tol) {
    return Point(std::numeric_limits<double>::signaling_NaN(),
                 std::numeric_limits<double>::signaling_NaN());
  }

  return Point(d * e1 - b * e2, -c * e1 + a * e2) / det;
}

/******************************************************************************/

bool isFeasible(const Inequalities& inequalities, const Point& point,
                double tol) {
  return ((inequalities.leftCols<2>() * point.transpose()).array() <=
          inequalities.col(2).array() + tol)
      .all();
}

/******************************************************************************/

}  // namespace lp2d
}  // namespace gtsam
