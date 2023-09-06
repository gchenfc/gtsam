#include "RetimingFactor.h"

#include "RetimingFactorGraph.h"

namespace gtsam {

RetimingFactor::RetimingFactor(const RetimingFactorGraph& factors)
    : Factor(factors.keyVector()),
      objectives_(factors.objectives()),
      equalities_(factors.equalities()),
      inequalities_(factors.inequalities()) {}

}  // namespace gtsam
