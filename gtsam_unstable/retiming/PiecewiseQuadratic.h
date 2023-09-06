/**
 * @file PiecewiseQuadratic.h
 * @brief A piecewise quadratic used to define objectives.
 * @author Gerry Chen
 * @date Sept 2023
 */

#pragma once

#include <memory>

namespace gtsam {

class PiecewiseQuadratic {
 public:
  using shared_ptr = std::shared_ptr<PiecewiseQuadratic>;
  using This = PiecewiseQuadratic;

  // TODO(gerry): implementation

  // Testable
  void print(const std::string& s = "Piecewise Quadratic",
             const KeyFormatter& formatter = DefaultKeyFormatter) const {
    std::cout << s << " unimplemented\n";
  }
  bool equals(const This& other, double tol = 1e-9) const {
    throw std::runtime_error("Not implemented");
  }
};

template <>
struct traits<PiecewiseQuadratic> : public Testable<PiecewiseQuadratic> {};

/// nullopt denotes "greedy" objective, GaussianFactor is a quadratic objective
using RetimingObjective = std::optional<PiecewiseQuadratic>;

// traits for Testable
template <>
struct traits<RetimingObjective> {
  static void Print(const RetimingObjective& m,
                    const std::string& str = "Objective") {
    if (m)
      m->print(str);
    else
      std::cout << str << " GREEDY\n";
  }
  static bool Equals(const RetimingObjective& m1, const RetimingObjective& m2,
                     double tol = 1e-8) {
    if (!m1) return !m2;
    return m1->equals(*m2, tol);
  }
};

}  // namespace gtsam
