#include "RetimingConditional.h"

#include "eliminate.h"

using Linears = gtsam::LinearConstraint::Linears;
using Linear = gtsam::LinearConstraint::Linear;

namespace gtsam {

Linears substituteLinears(const Linears& Ab, const KeyVector& keys,
                          const ScalarValues& parents) {
  if (Ab.cols() == 2) return Ab;

  Linear y(Ab.cols() - 2);
  for (int c = 1; c < Ab.cols() - 1; ++c) {
    y(c - 1) = parents.at(keys.at(c));
  }

  Linears res(Ab.rows(), 2);
  const auto& rhs =
      Ab.rightCols<1>() - Ab.middleCols(1, Ab.cols() - 2) * y.transpose();
  res << Ab.leftCols<1>(), rhs;
  return res;
}

double solveEqualities(const Linears& equalities, const KeyVector& keys,
                       const ScalarValues& parents) {
  auto res = substituteLinears(equalities, keys, parents);
  RetimingFactor::removeRedundantEqualitiesInplace(res);
  return res(0, 1) / res(0, 0);
}

double solveInequalitiesGreedily(const Linears& inequalities,
                                 const KeyVector& keys,
                                 const ScalarValues& parents) {
  auto res = substituteLinears(inequalities, keys, parents);
  RetimingFactor::removeRedundantInequalitiesInplace(res);
  // Return the upper bound
  assertm(res(0, 0) == 1, "No upper bound on inequality");
  return res(0, 1);
}

/******************************************************************************/

double RetimingConditional::solve(const ScalarValues& parents) {
  if (equalities().size()) {
    return solveEqualities(equalities(), keys(), parents);
  } else if (inequalities().size()) {
    if (elimination_helpers::AllObjectivesGreedy(objectives())) {
      return solveInequalitiesGreedily(inequalities(), keys(), parents);
    } else {
      throw std::runtime_error(
          "Solving inequality with non-greedy objectives not implemented yet");
    }
  }

  throw std::runtime_error("Solving objective-only not implemented yet");
  return -1;
}

}  // namespace gtsam
