#include "Lp2d.h"

namespace gtsam {
namespace lp2d {

/******************************************************************************/

ScalarBounds extremalsY(const Inequalities& inequalities) {
  return (ScalarBounds() << 1, 0, -1, 0).finished();  //
}

/******************************************************************************/

Point intersection(const Inequality& line1, const Inequality& line2) {
  // [a, b] . [x] = [e1]
  // [c, d]   [y] = [e2]
  // xy = Ainv * e = (1/(ad-bc)) * [d, -b; -c, a] * [e1; e2]

  const double &a = line1(0), &b = line1(1), &e1 = line1(2),
               /**/ &c = line2(0), &d = line2(1), &e2 = line2(2);

  double det = a * d - b * c;
  if (det == 0) throw std::runtime_error("Lines are parallel");

  return Point(d * e1 - b * e2, -c * e1 + a * e2) / det;
}

/******************************************************************************/

bool isFeasible(const Inequalities& inequalities, const Point& point) {
  return ((inequalities.leftCols<2>() * point.transpose()).array() <=
          inequalities.col(2).array())
      .all();
}

/******************************************************************************/

}  // namespace lp2d
}  // namespace gtsam