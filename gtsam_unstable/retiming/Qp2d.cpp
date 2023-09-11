#include "Qp2d.h"

namespace gtsam {
namespace qp2d {

PiecewiseLinear argmin(const PiecewiseQuadratic& objective,
                       const Inequalities& inequalities,
                       Bounds1d* bounds_on_argument) {
  return PiecewiseLinear{};
}

}  // namespace qp2d
}  // namespace gtsam
