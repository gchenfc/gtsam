#include "RetimingFactorGraph.h"

namespace gtsam {

using Linear = RetimingFactor::Linear;
using Linears = RetimingFactor::Linears;

RetimingObjectives RetimingFactorGraph::objectives() const {
  const KeyVector& keys = keyVector();
  RetimingObjectives objectives;
  for (const auto& factor : *this) {
    rekey_and_append(factor->objectives(), factor->keys(), keys, objectives);
  }
  return objectives;
}

Linears RetimingFactorGraph::equalities() const {
  const KeyVector& keys = keyVector();
  Linears equalities;
  for (const auto& factor : *this) {
    rekey_and_append(factor->equalities(), factor->keys(), keys, equalities);
  }
  return equalities;
}

Linears RetimingFactorGraph::inequalities() const {
  const KeyVector& keys = keyVector();
  Linears inequalities;
  for (const auto& factor : *this) {
    rekey_and_append(factor->inequalities(), factor->keys(), keys,
                     inequalities);
  }
  return inequalities;
}

}  // namespace gtsam
