/**
 * @file RetimingConditional.h
 * @brief Conditional for a retiming problem
 * @author Gerry Chen
 * @date Sept 2023
 */

#pragma once

#include <gtsam/inference/Conditional.h>

#include "RetimingFactor.h"
#include "utils.h"  // for ScalarValues

namespace gtsam {

/// @brief We will always lazy-evaluate the conditional, so RetimingConditional
/// can be defined by the factors that were eliminated to define it
class GTSAM_EXPORT RetimingConditional
    : public RetimingFactor,
      public Conditional<RetimingFactor, RetimingConditional> {
 public:
  using shared_ptr = std::shared_ptr<RetimingConditional>;
  using This = RetimingConditional;

  ScalarValues solve(const ScalarValues& parents) { return {{0, 0.0}}; }

  // Testable
  void print(
      const std::string& s = "Retiming Conditional",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override {
    std::cout << s << " defined by ";
    this->RetimingFactor::print("Retiming Factor", formatter);
  }
  bool equals(const This& other, double tol = 1e-9) const {
    return this->RetimingFactor::equals(other, tol);
  }
};

template <>
struct traits<RetimingConditional> : public Testable<RetimingConditional> {};

}  // namespace gtsam
