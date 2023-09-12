#include "Lp2d.h"

#include <iostream>

#include "assert.h"

namespace gtsam {
namespace lp2d {

/******************************************************************************/

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

bool isCcw(const Inequality& line1, const Inequality& line2) {
  // Check if the inequalities are going ccw or cw.  Return true for ccw
  return (line1(0) * line2(1) - line1(1) * line2(0)) > 0;
};

int findCcwIntersection(const Inequalities& inequalities, int start_index,
                        bool ccw = true) {
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
  if (findCcwIntersection(inequalities, cur_index, false) != end_index) {
    throw std::runtime_error("Not enough space allocated");
  }
  // No resizing needed
}

bool sortBoundaries(const Inequalities& inequalities, Inequalities& result) {
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
  if (start_index == inequalities.rows()) return false;  // infeasible

  // Now loop over the rest of the edges
  for (int index = start_index + 1; index < inequalities.rows(); ++index) {
    most_recent_added = findCcwIntersection(inequalities, most_recent_added);
    if (most_recent_added == -1) {
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
    } else if (most_recent_added == start_index) {
      // We've looped back to the start, so we're done
      result.conservativeResize(index + 1, Eigen::NoChange);
      return true;
    } else {
      // Insert the new inequality and move on
      result.row(index + 1) = inequalities.row(most_recent_added);
    }
  }

  return false;  // infeasible
}

/******************************************************************************/

/// @brief Insert new inequalities while maintaining sorted property
bool insertBoundariesSorted(const Inequalities& inequalities,
                            const Inequalities& new_inequalities,
                            Inequalities& result) {
  // TODO(gerry): implement this more efficiently
  Inequalities tmp(inequalities.rows() + new_inequalities.rows(), 3);
  tmp << inequalities, new_inequalities;
  return sortBoundaries(tmp, result);
}

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
