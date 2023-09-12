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

void PiecewiseQuadratic1d::MinInPlace(PiecewiseQuadratic1d& q1,
                                      const PiecewiseQuadratic1d& q2) {
  throw std::runtime_error("TODO");
  return;
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
namespace internal {
Eigen::Matrix<double, 1, 3> substitute(
    const Eigen::Ref<const Eigen::Matrix<double, 1, 6>>& q, const double m_,
    const double b_) {
  // x = m_*y + b_
  // x = K*y + k  (re-notate with K, k)
  // a.x^2 + b.y^2 + c.xy + d.x + e.y + f
  // = (a.K^2 + b + c.K).y^2 + (2.a.K.k + c.k + d.K + e).y + (a.k^2 + d.k + f)
  const double &a = q(0), &b = q(1), &c = q(2), &d = q(3), &e = q(4), &f = q(5);
  const double &K = m_, &k = b_;
  return Eigen::Matrix<double, 1, 3>((a * K * K + b + c * K),  // y^2
                                     (2 * a * K * k + c * k + d * K + e),  // y
                                     (a * k * k + d * k + f));             // 1
}
}  // namespace internal

/******************************************************************************/
// Calculates the number of segments in the piecewise quadratic result based on
// the number of intersections of the conditional and the piecewise segments.
int computeNumSegments(const Vec& xc, const PiecewiseLinear& conditional) {
  int count = 0;
  const auto& m = conditional.m;
  const auto& b = conditional.b;
  const auto& yc = conditional.xc;

  if (yc.size() == 0) return xc.size() + 1;

  auto countXcCrossings = [&xc](double x1, double x2) {
    // Imagine xc = 0   1   2   3   4
    //     x1, x2 =  0.5---------3.5
    // find x2 will give 4
    // find x1 will give 1
    // so find(x2) - find(x1) will count how many xc's we crossed
    // Use abs since we don't know which one is left/right
    return std::abs(std::distance(std::lower_bound(xc.begin(), xc.end(), x1),
                                  std::upper_bound(xc.begin(), xc.end(), x2)));
  };

  // The x value at the start of the yc segment
  double segment_x_start = (m(0) > 0) ? -std::numeric_limits<double>::infinity()
                                      : std::numeric_limits<double>::infinity();
  // Loop through each yc segment
  for (int i = 0; i < yc.size(); ++i) {
    // The x value at the end of the yc segment
    double segment_x_end = m(i) * yc(i) + b(i);
    // Add the number of times the segment crosses xc's
    // Note: segments = crossings + 1
    count += countXcCrossings(segment_x_start, segment_x_end) + 1;

    segment_x_start = segment_x_end;  // move on to next segment
  }

  // At the very end, we must do the very last yc segment
  double segment_x_end = (m(yc.size() - 1) > 0)
                             ? std::numeric_limits<double>::infinity()
                             : -std::numeric_limits<double>::infinity();
  count += countXcCrossings(segment_x_start, segment_x_end) + 1;

  return count;
}

/******************************************************************************/
void PiecewiseQuadratic::iterateOverXcYcSegments(
    const Vec& xc, const PiecewiseLinear& conditional,
    const std::function<void(int x_segment_index, int y_segment_index,
                             double y_upper_bound)>& func) {
  // This is computed by iterating over the y-regions and finding all the
  // x-crossings that occur to iterate over the x-regions.
  const auto& m = conditional.m;
  const auto& b = conditional.b;
  const auto& yc = conditional.xc;

  auto doYRegion = [&xc, &func, &m, &b, &yc](int y_segment_index, double x1,
                                             double x2) {
    // Intersection of segment yj with x is
    // x = my + b, so y = (x - b) / m
    auto intersect = [&m_ = m(y_segment_index), &b_ = b(y_segment_index)](
                         double x) { return (x - b_) / m_; };
    if (x1 < x2) {  // left-to-right
      auto it = std::lower_bound(xc.begin(), xc.end(), x1);
      for (; (it != xc.end()) && (*it < x2); ++it) {
        func(std::distance(xc.begin(), it), y_segment_index, intersect(*it));
      }
      func(std::distance(xc.begin(), it), y_segment_index,
           // could also do intersect(x2) but this feels less risky
           (y_segment_index == yc.size())
               ? std::numeric_limits<double>::infinity()
               : yc(y_segment_index));
    } else {  // right-to-left
      auto it_right = std::upper_bound(xc.begin(), xc.end(), x1);
      auto it_left = it_right - 1;
      for (; (it_left >= xc.begin()) && (*it_left > x2);
           --it_left, --it_right) {
        func(std::distance(xc.begin(), it_right), y_segment_index,
             intersect(*it_left));
      }
      func(std::distance(xc.begin(), it_right), y_segment_index,
           (y_segment_index == yc.size())
               ? std::numeric_limits<double>::infinity()
               : yc(y_segment_index));
    }
  };

  if (yc.size() == 0) {  // only 1 y-segment
    doYRegion(0, -std::numeric_limits<double>::infinity(),
              std::numeric_limits<double>::infinity());
    return;
  };

  // The x value at the start of the yc segment
  // double segment_x_start = (m(0) > 0) ?
  // -std::numeric_limits<double>::infinity()
  //                                     :
  //                                     std::numeric_limits<double>::infinity();
  double prev_y = -std::numeric_limits<double>::infinity();
  // Loop through each yc segment
  for (int i = 0; i < yc.size(); ++i) {
    // The x value at the end of the yc segment
    double segment_x_start = m(i) * prev_y + b(i);
    double segment_x_end = m(i) * yc(i) + b(i);
    doYRegion(i, segment_x_start, segment_x_end);
    prev_y = yc(i);
  }

  // At the very end, we must do the very last yc segment
  double segment_x_start = m.tail<1>().value() * prev_y + b.tail<1>().value();
  double segment_x_end =
      m.tail<1>().value() * std::numeric_limits<double>::infinity();
  doYRegion(yc.size(), segment_x_start, segment_x_end);
}

/******************************************************************************/

PiecewiseQuadratic1d PiecewiseQuadratic::substitute(
    const PiecewiseLinear& conditional) const {
  // `this` is a piecewise quadratic q(x, y), which is piecewise in x
  const auto& Q = C_;
  const auto& xc = xc_;
  // `conditional` is a piecewise linear x(y), which is piecewise in y
  const auto& m = conditional.m;
  const auto& b = conditional.b;

  // To substitute, we work our way up the segments of the conditional.
  // Each time we cross a segment boundary in q, we need to split off and add
  // another segment.
  std::vector<Eigen::Matrix<double, 1, 3>> Qs;  // the new segments of q
  std::vector<double> ycs;  // the new segment boundaries of q

  auto func = [&ycs, &Qs, &Q, &m, &b](int x_segment_index, int y_segment_index,
                                      double yc) {
    ycs.push_back(yc);
    Qs.push_back(::gtsam::internal::substitute(
        Q.row(x_segment_index), m(y_segment_index), b(y_segment_index)));
  };
  iterateOverXcYcSegments(xc, conditional, func);

  // Extract into Eigen types
  PiecewiseQuadratic1d result;
  result.C.resize(Qs.size(), 6);
  result.xc.resize(ycs.size() - 1);  // final yc should be inf
  std::copy(Qs.begin(), Qs.end(), result.C.rowwise().begin());
  std::copy(ycs.begin(), ycs.end() - 1, result.xc.begin());
  return result;
}

}  // namespace gtsam
