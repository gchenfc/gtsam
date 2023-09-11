#include "PiecewiseQuadratic.h"

namespace gtsam {

using Bounds1d = PiecewiseQuadratic::Bounds1d;

/******************************************************************************/

PiecewiseQuadratic::PiecewiseQuadratic(
    const std::vector<RetimingObjective>& objectives) {}

/******************************************************************************/

PiecewiseQuadratic::PiecewiseQuadratic(
    const std::vector<PiecewiseQuadratic>& objectives) {}

/******************************************************************************/

double PiecewiseQuadratic::evaluate(double x, double y) const { return 0.0; }

/******************************************************************************/

std::pair<PiecewiseLinear, Bounds1d> PiecewiseQuadratic::solveParametric(
    const Inequalities& inequalities) const {
  return {PiecewiseLinear(), Bounds1d()};
}

/******************************************************************************/

PiecewiseQuadratic PiecewiseQuadratic::substitute(
    const PiecewiseLinear& conditional) const {
  return PiecewiseQuadratic();
}

}  // namespace gtsam
