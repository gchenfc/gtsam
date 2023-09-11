#include "Qp2d.h"

namespace gtsam {
namespace qp2d {

PiecewiseQuadratic1d min(const PiecewiseQuadratic& objective,
                         const Inequalities& inequalities,
                         Bounds1d* bounds_on_argument) {
  return PiecewiseQuadratic1d{};
}

}  // namespace qp2d
}  // namespace gtsam
