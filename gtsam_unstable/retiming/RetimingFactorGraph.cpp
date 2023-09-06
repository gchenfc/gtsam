#include <numeric>

#include "RetimingFactorGraph.h"

#include <gtsam/inference/EliminateableFactorGraph-inst.h>

#include "RetimingFactor.h"
#include "RetimingConditional.h"
#include "RetimingBayesNet.h"
#include "RetimingTrees.h"

namespace gtsam {

// Instantiate base classes
template class FactorGraph<RetimingFactor>;
template class EliminateableFactorGraph<RetimingFactorGraph>;

using Linear = RetimingFactor::Linear;
using Linears = RetimingFactor::Linears;

RetimingObjectives RetimingFactorGraph::objectives(const KeyVector& kv) const {
  RetimingObjectives objectives;
  for (const auto& factor : *this) {
    rekey_and_append(factor->objectives(), factor->keys(), kv, objectives);
  }
  return objectives;
}

Linears RetimingFactorGraph::equalities(const KeyVector& kv) const {
  auto numRows =
      std::accumulate(begin(), end(), 0, [](auto sum, const auto& factor) {
        return sum + factor->equalities().rows();
      });
  Linears equalities(numRows, kv.size() + 1);
  size_t row = 0;
  for (const auto& factor : *this) {
    equalities.middleRows(row, factor->equalities().rows()) =
        LinearConstraint::rekey(factor->equalities(), factor->keys(), kv);
    row += factor->equalities().rows();
  }
  return equalities;
}

Linears RetimingFactorGraph::inequalities(const KeyVector& kv) const {
  auto numRows =
      std::accumulate(begin(), end(), 0, [](auto sum, const auto& factor) {
        return sum + factor->inequalities().rows();
      });
  Linears inequalities(numRows, kv.size() + 1);
  size_t row = 0;
  for (const auto& factor : *this) {
    inequalities.middleRows(row, factor->inequalities().rows()) =
        LinearConstraint::rekey(factor->inequalities(), factor->keys(), kv);
    row += factor->inequalities().rows();
  }
  return inequalities;
}

}  // namespace gtsam
