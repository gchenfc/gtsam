#include "Lp2d.h"

#include <iostream>

namespace gtsam {
namespace lp2d {

/******************************************************************************/

ScalarBounds extremalsY(const Inequalities& inequalities) {
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

Point intersection(const Inequality& line1, const Inequality& line2,
                   double parallel_tol) {
  // [a, b] . [x] = [e1]
  // [c, d]   [y] = [e2]
  // xy = Ainv * e = (1/(ad-bc)) * [d, -b; -c, a] * [e1; e2]

  const double &a = line1(0), &b = line1(1), &e1 = line1(2),
               /**/ &c = line2(0), &d = line2(1), &e2 = line2(2);

  double det = a * d - b * c;
  assert(abs(det) >= parallel_tol && "Lines are parallel");

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
