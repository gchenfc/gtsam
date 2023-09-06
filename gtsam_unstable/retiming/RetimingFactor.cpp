#include "RetimingFactor.h"

#include "RetimingFactorGraph.h"

namespace gtsam {

RetimingFactor::RetimingFactor(const RetimingFactorGraph& factors)
    : Factor(factors.keyVector()),
      objectives_(factors.objectives()),
      equalities_(factors.equalities()),
      inequalities_(factors.inequalities()) {}

/******************************************************************************/

RetimingFactor::shared_ptr RetimingFactor::substitute(
    const size_t& keyIndex, const Linear& equality) const {
  RetimingFactor::shared_ptr result = std::make_shared<RetimingFactor>(
      keys_, RetimingObjectives{}, Linears{}, Linears{});

  result->keys_.erase(result->keys_.begin() + keyIndex);

  result->objectives_ = objectives_;  // TODO(gerry)
  for (auto eq : equalities_) {
    double factor = -eq.A.at(keyIndex) / equality.A.at(keyIndex);
    std::transform(equality.A.begin(), equality.A.end(), eq.A.begin(),
                   eq.A.begin(),
                   [factor](double x, double y) { return y + x * factor; });
    eq.b += factor * equality.b;
    eq.A.erase(eq.A.begin() + keyIndex);
    result->equalities_.push_back(eq);
  }
  for (auto ineq : inequalities_) {
    double factor = -ineq.A.at(keyIndex) / equality.A.at(keyIndex);
    std::transform(equality.A.begin(), equality.A.end(), ineq.A.begin(),
                   ineq.A.begin(),
                   [factor](double x, double y) { return y + x * factor; });
    ineq.b += factor * equality.b;
    ineq.A.erase(ineq.A.begin() + keyIndex);
    result->inequalities_.push_back(ineq);
  }
  return result;
}

}  // namespace gtsam
