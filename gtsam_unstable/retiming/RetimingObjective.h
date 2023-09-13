/**
 * @file RetimingObjective.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-09-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include "PiecewiseQuadratic.h"

namespace gtsam {

/// RetimingObjective represents either a "greedy" or quadratic objective.
struct RetimingObjective {
  bool isGreedy;
  PiecewiseQuadratic quadratic;

  RetimingObjective() : isGreedy(true) {}
  RetimingObjective(const PiecewiseQuadratic& quadratic)
      : isGreedy(false), quadratic(quadratic) {}

  // Static convenience constructor
  static RetimingObjective Greedy() { return RetimingObjective(); }

  // Rekey
  RetimingObjective rekey(const KeyVector& src_keys,
                          const KeyVector& dest_keys) const {
    if (isGreedy) return *this;
    return RetimingObjective(quadratic.rekey(src_keys, dest_keys));
  }

  // Testable
  void print(const std::string& s = "Objective",
             const KeyFormatter& formatter = DefaultKeyFormatter) const {
    if (isGreedy) {
      std::cout << s << " GREEDY\n";
    } else {
      quadratic.print(s);
    }
  }
  bool equals(const RetimingObjective& other, double tol = 1e-9) const {
    if (isGreedy) return other.isGreedy;
    return !other.isGreedy && quadratic.equals(other.quadratic, tol);
  }
};

using RetimingObjectives = std::vector<RetimingObjective>;

template <typename T>
RetimingObjectives substitute(const RetimingObjectives& objectives,
                              const T& equality) {
  RetimingObjectives result;
  result.reserve(objectives.size());
  std::transform(objectives.begin(), objectives.end(),
                 std::back_inserter(result),
                 [equality](const RetimingObjective& objective) {
                   return objective.isGreedy
                              ? objective
                              : RetimingObjective(
                                    objective.quadratic.substitute(equality));
                 });
  auto it = std::remove_if(
      result.begin(), result.end(), [](const RetimingObjective& objective) {
        return objective.isGreedy || objective.quadratic.empty();
      });
  result.erase(it, result.end());
  return result;
}

// traits for Testable
template <>
struct traits<RetimingObjective> : public Testable<RetimingObjective> {};

}  // namespace gtsam
