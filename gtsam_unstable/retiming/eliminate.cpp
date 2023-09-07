#include "eliminate.h"

#include <gtsam/inference/Ordering.h>
#include "RetimingConditional.h"
#include "RetimingFactorGraph.h"
#include "Lp2d.h"

namespace gtsam {

using Linears = LinearConstraint::Linears;

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

  // Next, check for 2-variable inequalities
  if (ordering.size() == 2) {
    // TODO(gerry): use LP solver to find the optimal solution
    if (!std::all_of(
            factor.objectives().begin(), factor.objectives().end(),
            [](const auto& objective) { return objective.isGreedy; })) {
      throw std::runtime_error(
          "Elimination with non-greedy objectives not yet implemented");
    }
    return {/*Conditional*/ std::make_shared<RetimingConditional>(factor),
            /*   Joint   */ std::make_shared<RetimingFactor>(
                KeyVector{ordering.back()}, factor.objectives(),
                LinearConstraint::dropCol(factor.equalities(), col_index),
                lp2d::extremalsY(factor.inequalities()))};
  }

  // Or check if there are more than 2 variables, but only 2 have inequalities
  // (edge case)
  const auto& Ab = factor.inequalities();
  if (Ab.leftCols(Ab.cols() - 1).colwise().any().sum() <= 2) {
    // Make sure all objectives are greedy
    if (!std::all_of(
            factor.objectives().begin(), factor.objectives().end(),
            [](const auto& objective) { return objective.isGreedy; })) {
      throw std::runtime_error(
          "Elimination with non-greedy objectives not yet implemented");
    }

    // select just the 2 columns with non-zero coefficients, and assign to `tmp`
    std::array<int, 2> nnz_cols;
    int tmp_col = 0;
    for (int col = 0; (col < Ab.cols() - 1); ++col) {
      if (Ab.col(col).any()) nnz_cols.at(tmp_col++) = col;
    }
    if (std::get<0>(nnz_cols) != col_index) {
      throw std::runtime_error(
          "Not inequality type - must be objective which is not implemented.");
    };
    lp2d::Inequalities tmp(Ab.rows(), 3);
    tmp << Ab.col(nnz_cols.at(0)), Ab.col(nnz_cols.at(1)), Ab.rightCols<1>();
    // Solve for the extremals of the second column of nonzeros
    auto tmp_extremals = lp2d::extremalsY(tmp);

    // Insert the extremals into a correctly shaped inequalities matrix
    Linears inequalities = Linears::Zero(2, Ab.cols() - 1);
    inequalities.col(std::get<1>(nnz_cols) - 1) = tmp_extremals.leftCols<1>();
    inequalities.rightCols<1>() = tmp_extremals.rightCols<1>();

    // return the conditional and joint
    ordering.erase(ordering.begin());
    return {/*Conditional*/ std::make_shared<RetimingConditional>(
                KeyVector{key, ordering.at(std::get<1>(nnz_cols) - 1)},
                factor.objectives(), Linears(0, 3), tmp),
            /*   Joint   */ std::make_shared<RetimingFactor>(
                ordering, factor.objectives(),
                factor.equalities().rightCols(factor.equalities().cols() - 1),
                inequalities)};
  }
  std::cout << "Ordering size is " << ordering.size() << std::endl;

  throw std::runtime_error(
      "Elimination on inequality factors >2 keys and elimination on objectives "
      "not implemented");

  return {nullptr, nullptr};
}

}  // namespace gtsam
