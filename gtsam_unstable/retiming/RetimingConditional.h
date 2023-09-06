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

  // Constructors
  using RetimingFactor::RetimingFactor;
  // RetimingConditional(const RetimingFactor& factor, size_t nrFrontals = 0)
  //     : RetimingFactor(factor), Conditional(nrFrontals) {}

  static shared_ptr Equality(const KeyVector& keys, const Linear& equality) {
    return std::make_shared<RetimingConditional>(keys, RetimingObjectives{},
                                                 Linears{equality}, Linears{});
  }

  // Implementation from Conditional

  /**
   * All conditional types need to implement a `logProbability` function, for
   * which exp(logProbability(x)) = evaluate(x).
   */
  double logProbability(const HybridValues& c) const override { return 0.0; };

  /**
   * All conditional types need to implement an `evaluate` function, that yields
   * a true probability. The default implementation just exponentiates
   * logProbability.
   */
  double evaluate(const HybridValues& c) const override { return 0.0; };

  /**
   * All conditional types need to implement a log normalization constant to
   * make it such that error>=0.
   */
  double logNormalizationConstant() const override { return 0.0; };

  // TODO!!!
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
