#include "PiecewiseQuadratic.h"

#include "RetimingObjective.h"

namespace gtsam {

using Bounds1d = PiecewiseQuadratic::Bounds1d;
using Mat = PiecewiseQuadratic::Mat;
using Vec = PiecewiseQuadratic::Vec;

/******************************************************************************/

/// Add another objective, in-place
void AddInPlace(PiecewiseQuadratic& objective1,
                const PiecewiseQuadratic& objective2) {
  const Mat C1 = objective1.C();
  const Vec xc1 = objective1.xc();
  const Mat& C2 = objective2.C();
  const Vec& xc2 = objective2.xc();
  Mat& C = objective1.C();
  Vec& xc = objective1.xc();

  C.resize(C1.rows() + C2.rows() - 1, 6);
  xc.resize(xc1.size() + xc2.size());

  size_t i1 = 0, i2 = 0, i = 0;
  while (i1 < xc1.rows() && i2 < xc2.rows()) {
    C.row(i) = C1.row(i1) + C2.row(i2);
    if (xc1(i1) < xc2(i2)) {
      xc(i) = xc1(i1);
      ++i1;
    } else {
      xc(i) = xc2(i2);
      ++i2;
    }
    ++i;
  }
  if (i1 == xc1.rows()) {
    C.bottomRows(C2.rows() - i2) = C2.bottomRows(C2.rows() - i2);
    C.bottomRows(C2.rows() - i2).rowwise() += C1.bottomRows<1>();
    xc.tail(xc2.size() - i2) = xc2.tail(xc2.size() - i2);
  } else {
    C.bottomRows(C1.rows() - i1) = C1.bottomRows(C1.rows() - i1);
    C.bottomRows(C1.rows() - i1).rowwise() += C2.bottomRows<1>();
    xc.tail(xc1.size() - i1) = xc1.tail(xc1.size() - i1);
  }
}

/******************************************************************************/

PiecewiseQuadratic::PiecewiseQuadratic(
    const std::vector<RetimingObjective>& objectives)
    : PiecewiseQuadratic(objectives.size() == 0 ? PiecewiseQuadratic()
                                                : objectives.at(0).quadratic) {
  for (int i = 1; i < objectives.size(); ++i) {
    AddInPlace(*this, objectives.at(i).quadratic);
  }
}

/******************************************************************************/

PiecewiseQuadratic::PiecewiseQuadratic(
    const std::vector<PiecewiseQuadratic>& objectives)
    : PiecewiseQuadratic(objectives.size() == 0 ? PiecewiseQuadratic()
                                                : objectives.at(0)) {
  for (int i = 1; i < objectives.size(); ++i) {
    AddInPlace(*this, objectives.at(i));
  }
}

/******************************************************************************/

double PiecewiseQuadratic::evaluate(double x, double y) const {
  auto i = findIndex(x);
  return a()(i) * x * x + b()(i) * y * y + c()(i) * x * y + d()(i) * x +
         e()(i) * y + f()(i);
}

/******************************************************************************/

std::pair<PiecewiseQuadratic, Bounds1d> PiecewiseQuadratic::solveParametric(
    const Inequalities& inequalities) const {
  return {PiecewiseQuadratic(), Bounds1d()};
}

/******************************************************************************/

PiecewiseQuadratic PiecewiseQuadratic::substitute(
    const PiecewiseLinear& conditional) const {
  return *this;
}

}  // namespace gtsam
