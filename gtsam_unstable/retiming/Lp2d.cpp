#include "Lp2d.h"

namespace gtsam {
namespace lp2d {

/******************************************************************************/

Inequalities extremalsY(const Inequalities& inequalities) {
  return (Inequalities() << 1, 0, -1, 0).finished();  //
}

/******************************************************************************/

Point intersection(const Inequality& line1, const Inequality& line2) {
  return {0, 0};  //
}

/******************************************************************************/

}  // namespace lp2d
}  // namespace gtsam