#include "eliminate.h"

#include <gtsam/inference/Ordering.h>

namespace gtsam {

std::pair<std::shared_ptr<RetimingConditional>, std::shared_ptr<RetimingFactor>>
EliminateRetiming(const RetimingFactorGraph& factors, const Ordering& keys) {
  if (keys.size() != 1) {
    throw std::runtime_error("Multi-key elimination not Implemented");
  }
  const auto& key = keys.at(0);

  // Collect all the objectives/constraints into a single factor
  RetimingFactor factor(factors);

  // First search through all the equalities
  for (const auto& equality : factor.equalities()) {
    const auto& eq_cond = SolveEquality(factor, key);
    if (!eq_cond) continue;
    // factor.eraseEquality(eq_cond);  // TODO(gerry): fix this
    // factors.substitute(eq_cond);    // TODO(gerry): implement this
  }

  return {nullptr, nullptr};
}

std::optional<EqualityConditional> SolveEquality(const RetimingFactor& factor,
                                                 const Key& key) {
  // Find the index of the key in the factor
  const auto& fkeys = factor.keys();
  const auto& key_it = std::find(fkeys.begin(), fkeys.end(), key);
  if (key_it == fkeys.end()) {
    std::cout << "WARNING: There was a factor in Dense Eliminate that didn't "
                 "contain the key!!!"
              << std::endl;
    return std::nullopt;
  }
  const auto& key_index = std::distance(fkeys.begin(), key_it);

  // Check if any of the inequalities have non-zero coefficient
  for (const auto& eq : factor.equalities()) {
    double cx = eq.A.at(key_index);
    if (cx != 0) {
      ScalarValues cy;
      for (const auto& key2 : factor.keys()) {
        if (key2 != key) cy.insert({key2, eq.A.at(key2) / cx});
      }
      return EqualityConditional{eq.b, cy};
    }
  }
  return std::nullopt;
}

}  // namespace gtsam
