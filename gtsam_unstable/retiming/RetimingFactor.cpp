#include "RetimingFactor.h"

#include "RetimingFactorGraph.h"

namespace gtsam {

RetimingFactor::RetimingFactor(const RetimingFactorGraph& factors)
    : RetimingFactor(factors, factors.keyVector()) {}

RetimingFactor::RetimingFactor(const RetimingFactorGraph& factors,
                               const KeyVector& ordering)
    : Factor(ordering),
      objectives_(factors.objectives(ordering)),
      equalities_(factors.equalities(ordering)),
      inequalities_(factors.inequalities(ordering)) {}

/******************************************************************************/

RetimingFactor::shared_ptr RetimingFactor::substitute(
    const size_t& keyIndex, const Linear& equality) const {
  RetimingFactor::shared_ptr result = std::make_shared<RetimingFactor>(
      keys_,
      objectives_,  // TODO(gerry): update the objectives
      LinearConstraint::eliminate(equalities_, equality, keyIndex),
      LinearConstraint::eliminate(inequalities_, equality, keyIndex));

  result->keys_.erase(result->keys_.begin() + keyIndex);

  return result;
}

}  // namespace gtsam
