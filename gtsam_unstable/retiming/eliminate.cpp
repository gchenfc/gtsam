#include "eliminate.h"

#include <gtsam/inference/Ordering.h>
#include "RetimingConditional.h"
#include "RetimingFactorGraph.h"
#include "Lp2d.h"
#include "PiecewiseLinear.h"
#include "PiecewiseQuadratic.h"

namespace gtsam {

using namespace elimination_helpers;
using Linears = LinearConstraint::Linears;

/******************************************************************************/

inline std::pair<std::shared_ptr<RetimingConditional>,
                 std::shared_ptr<RetimingFactor>>
EliminateRetiming_(const RetimingFactorGraph& factors, const Ordering& keys);

std::pair<std::shared_ptr<RetimingConditional>, std::shared_ptr<RetimingFactor>>
EliminateRetiming(const RetimingFactorGraph& factors, const Ordering& keys) {
  auto [conditional, joint] = EliminateRetiming_(factors, keys);
#if PRINT_VERBOSITY > 15
  if (joint) joint->print("Passing on joint:");
#endif
  return {conditional, joint};
}

inline std::pair<std::shared_ptr<RetimingConditional>,
                 std::shared_ptr<RetimingFactor>>
EliminateRetiming_(const RetimingFactorGraph& factors, const Ordering& keys) {
  if (keys.size() != 1) {
    throw std::runtime_error("Multi-key elimination not Implemented");
  }
  const auto& key = keys.at(0);

#if PRINT_VERBOSITY > 10
  printf("Eliminating key %s", DefaultKeyFormatter(key).c_str());
#endif

  // Create an ordering with the key at the front.  We could use the functions
  // in Ordering.h but that's overkill since we only care about the first key
  KeyVector ordering = factors.keyVector();
  {  // Swap the key to the front
    const auto& keyIt = std::find(ordering.begin(), ordering.end(), key);
    assertm(keyIt != ordering.end(), "Key not found in ordering");
    auto keyIndex = std::distance(ordering.begin(), keyIt);
    ordering.at(keyIndex) = ordering.at(0);
    ordering.at(0) = key;
  }
  constexpr int col_index = 0;

  // Collect all the objectives/constraints into a single factor
  auto factor = std::make_shared<RetimingFactor>(factors, ordering);

  // First the trivial case
  if (ordering.size() == 1) {
    RetimingFactor::normalizeEqualitiesInplace(factor->equalities());
    RetimingFactor::normalizeInequalitiesInplace(factor->inequalities());
    return {
        /*Conditional*/ std::static_pointer_cast<RetimingConditional>(factor),
        /*   Joint   */ std::make_shared<RetimingFactor>()};
  }

  // Second check if there are any equalities that can be used for elimination
  const auto& equalities = factor->equalities();
  if (equalities.leftCols<1>().any()) {
    // Gauss-Jordan elimination on the first column
    for (int r = 0; r < equalities.rows(); ++r) {
      if (equalities(r, col_index) != 0) {
#if PRINT_VERBOSITY > 10
        printf("Eliminating with equality!!!\n");
#endif
        const LinearConstraint::Linear equality =
            equalities.row(r) / equalities(r, col_index);
        // Remove equality from factor, since it's now redundant
        factor->equalities() = LinearConstraint::dropRow(equalities, r);
        return {/*Conditional*/ RetimingConditional::Equality(factor->keys(),
                                                              equality),
                /*   Joint   */ factor->substitute(col_index, equality)};
      }
    }
  }

  // Third, check for 2-variable inequalities
  if (ordering.size() == 2) {
    return AllObjectivesGreedy(factor->objectives())
               ? EliminateLp2d(factor, ordering)
               : EliminateQp2d(factor, ordering);
  }
  // Or check if there are more than 2 variables, but only 2 have inequalities
  // (edge case)
  const auto& Ab = factor->inequalities();
  if (Ab.leftCols(Ab.cols() - 1).colwise().any().sum() <= 2) {
    return EliminateManyVars2Inequalities(*factor, ordering);
  }

  // Otherwise, the rest of the cases are not implemented
  throw std::runtime_error("Elimination on inequality factors " +
                           std::to_string(ordering.size()) +
                           ">2 keys and elimination on objectives "
                           "not implemented");

  return {nullptr, nullptr};
}

/******************************************************************************/
namespace elimination_helpers {
/******************************************************************************/

bool AllObjectivesGreedy(const RetimingObjectives& objectives) {
  return std::all_of(objectives.begin(), objectives.end(),
                     [](const auto& objective) { return objective.isGreedy; });
}

/******************************************************************************/

GTSAM_EXPORT std::pair<std::shared_ptr<RetimingConditional>,
                       std::shared_ptr<RetimingFactor>>
EliminateLp2d(const RetimingFactor::shared_ptr& factor,
              const KeyVector& ordering) {
  static constexpr int col_index = 0;
  assertm(AllObjectivesGreedy(factor->objectives()),
          "Elimination with non-greedy objectives not yet implemented");

  return {/*Conditional*/ std::static_pointer_cast<RetimingConditional>(factor),
          /*   Joint   */ std::make_shared<RetimingFactor>(
              KeyVector{ordering.back()}, factor->objectives(),
              LinearConstraint::dropCol(factor->equalities(), col_index),
              lp2d::extremalsY(factor->inequalities()))};
}

/******************************************************************************/

GTSAM_EXPORT std::pair<std::shared_ptr<RetimingConditional>,
                       std::shared_ptr<RetimingFactor>>
EliminateQp2d(const RetimingFactor::shared_ptr& factor, KeyVector& keys) {
  // static constexpr int col_index = 0;

#if PRINT_VERBOSITY > 40
  factor->print("EliminateQp2d, input is: ");
#endif

  PiecewiseQuadratic objective{factor->objectives()};

  const auto [/*PiecewiseQuadratic*/ new_objective,
              /*Inequalities*/ new_constraint] =
      objective.solveParametric(factor->inequalities());

#if PRINT_VERBOSITY > 32
  new_objective.print("EliminateQp2d: New objective!!!");
  std::cout << "New constraint!" << new_constraint << std::endl;
#endif

  // Even though we computed conditional, gtsam conditionals must derive from
  // factor (makes sense) but this will be prickly so let's just use the
  // original factor like with Lp2d
  return {/*Conditional*/ std::static_pointer_cast<RetimingConditional>(factor),
          /*   Joint   */ std::make_shared<RetimingFactor>(
              KeyVector{keys.back()},
              RetimingObjectives{RetimingObjective(new_objective)},
              factor->equalities(), new_constraint)};
}

/******************************************************************************/

GTSAM_EXPORT std::pair<std::shared_ptr<RetimingConditional>,
                       std::shared_ptr<RetimingFactor>>
EliminateManyVars2Inequalities(const RetimingFactor& factor,
                               KeyVector& ordering) {
  static constexpr int col_index = 0;
  const auto& Ab = factor.inequalities();

  // Make sure all objectives are greedy
  assertm(AllObjectivesGreedy(factor.objectives()),
          "Elimination with non-greedy objectives not yet implemented");

  // select just the 2 columns with non-zero coefficients, and assign to `tmp`
  std::array<int, 2> nnz_cols;
  int tmp_col = 0;
  for (int col = 0; (col < Ab.cols() - 1); ++col) {
    if (Ab.col(col).any()) nnz_cols.at(tmp_col++) = col;
  }
  assertm(std::get<0>(nnz_cols) == col_index,
          "Not inequality type - must be objective which is not implemented.");
  lp2d::Inequalities tmp(Ab.rows(), 3);
  tmp << Ab.col(nnz_cols.at(0)), Ab.col(nnz_cols.at(1)), Ab.rightCols<1>();
  // Solve for the extremals of the second column of nonzeros
  auto tmp_extremals = lp2d::extremalsY(tmp);

  // Insert the extremals into a correctly shaped inequalities matrix
  Linears inequalities = Linears::Zero(2, Ab.cols() - 1);
  inequalities.col(std::get<1>(nnz_cols) - 1) = tmp_extremals.leftCols<1>();
  inequalities.rightCols<1>() = tmp_extremals.rightCols<1>();

  // return the conditional and joint
  const auto& key = ordering.front();
  ordering.erase(ordering.begin());
  return {/*Conditional*/ std::make_shared<RetimingConditional>(
              KeyVector{key, ordering.at(std::get<1>(nnz_cols) - 1)},
              factor.objectives(), Linears(0, 3), tmp),
          /*   Joint   */ std::make_shared<RetimingFactor>(
              ordering, factor.objectives(),
              factor.equalities().rightCols(factor.equalities().cols() - 1),
              inequalities)};
}

}  // namespace elimination_helpers

}  // namespace gtsam
