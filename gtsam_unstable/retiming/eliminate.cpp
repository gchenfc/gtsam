#include "eliminate.h"

#include <gtsam/inference/Ordering.h>
#include "RetimingConditional.h"

namespace gtsam {

std::pair<std::shared_ptr<RetimingConditional>, std::shared_ptr<RetimingFactor>>
EliminateRetiming(const RetimingFactorGraph& factors, const Ordering& keys) {
  if (keys.size() != 1) {
    throw std::runtime_error("Multi-key elimination not Implemented");
  }
  const auto& key = keys.at(0);

  // Collect all the objectives/constraints into a single factor
  RetimingFactor factor(factors);

  // Find where the key is in the new combined factor
  const auto& keyIt = factor.find(key);
  if (keyIt == factor.end()) {
    throw std::runtime_error("Key not found in factor");
  }
  auto keyIndex = std::distance(std::as_const(factor).begin(), keyIt);

  // First search through all the equalities
  const auto& equalities = factor.equalities();
  auto it = equalities.begin();
  for (; it != equalities.end(); ++it) {
    if (it->A.at(keyIndex) != 0) {
      RetimingFactor::Linear cond = *it;  // Make a copy
      factor.equalities().erase(it);
      return {
          /*Conditional*/ RetimingConditional::Equality(factor.keys(), cond),
          /*   Joint   */ factor.substitute(keyIndex, cond)};
    }
  }

  return {nullptr, nullptr};
}

}  // namespace gtsam
