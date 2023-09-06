#include "eliminate.h"

#include <gtsam/inference/Ordering.h>
#include "RetimingConditional.h"
#include "RetimingFactorGraph.h"

namespace gtsam {

std::pair<std::shared_ptr<RetimingConditional>, std::shared_ptr<RetimingFactor>>
EliminateRetiming(const RetimingFactorGraph& factors, const Ordering& keys) {
  if (keys.size() != 1) {
    throw std::runtime_error("Multi-key elimination not Implemented");
  }
  const auto& key = keys.at(0);

  // Create an ordering with the key at the front.  We could use the functions
  // in Ordering.h but that's overkill since we only care about the first key
  KeyVector ordering = factors.keyVector();
  {  // Swap the key to the front
    const auto& keyIt = std::find(ordering.begin(), ordering.end(), key);
    if (keyIt == ordering.end()) {
      throw std::runtime_error("Key not found in ordering");
    }
    auto keyIndex = std::distance(ordering.begin(), keyIt);
    ordering.at(keyIndex) = ordering.at(0);
    ordering.at(0) = key;
  }
  constexpr int col_index = 0;

  // Collect all the objectives/constraints into a single factor
  RetimingFactor factor(factors, ordering);

  // First check if there are any equalities that can be used for elimination
  const auto& equalities = factor.equalities();
  if (equalities.leftCols<1>().any()) {
    // Gauss-Jordan elimination on the first column
    for (int r = 0; r < equalities.rows(); ++r) {
      if (equalities(r, col_index) != 0) {
        const LinearConstraint::Linear equality =
            equalities.row(r) / equalities(r, col_index);
        // Remove equality from factor, since it's now redundant
        factor.equalities() = LinearConstraint::dropRow(equalities, r);
        return {/*Conditional*/ RetimingConditional::Equality(factor.keys(),
                                                              equality),
                /*   Joint   */ factor.substitute(col_index, equality)};
      }
    }
  }

  return {nullptr, nullptr};
}

}  // namespace gtsam
