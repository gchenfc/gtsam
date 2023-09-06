#include "RetimingFactor.h"

#include "RetimingFactorGraph.h"

namespace gtsam {

RetimingFactor::RetimingFactor(const RetimingFactorGraph& factors,
                               bool removeRedundantConstraints,
                               bool checkForInfeasibility)
    : RetimingFactor(factors, factors.keyVector(), removeRedundantConstraints,
                     checkForInfeasibility) {}

RetimingFactor::RetimingFactor(const RetimingFactorGraph& factors,
                               const KeyVector& ordering,
                               bool removeRedundantConstraints,
                               bool checkForInfeasibility)
    : RetimingFactor(ordering, factors.objectives(ordering),
                     factors.equalities(ordering),
                     factors.inequalities(ordering), removeRedundantConstraints,
                     checkForInfeasibility) {}

/******************************************************************************/

void RetimingFactor::removeRedundantEqualitiesInplace(
    Matrix& Ab, bool checkForInfeasibility) {
  // Actually in retrospect inplace isn't really saving much because we need to
  // copy on resize anyway
  Eigen::HouseholderQR<Matrix> qr(Ab);

  Ab = qr.matrixQR().topRows(std::min(Ab.cols(), Ab.rows()));  // Copy!
  Ab.triangularView<Eigen::StrictlyLower>().setZero();         // Clear out Q

  if (checkForInfeasibility) {
    for (int r = Ab.rows() - 1; r >= 0; --r) {
      if ((Ab.row(r).head(Ab.cols() - 1).array().abs() < 1e-12).all()) {
        if (abs(Ab(r, Ab.cols() - 1)) >= 1e-12) {
          throw std::runtime_error("Infeasible");
        }
      } else {
        break;  // No more zero rows
      }
    }
  }
}

/******************************************************************************/

void RetimingFactor::removeRedundantInequalitiesInplace(Matrix& inequalities) {
  if (inequalities.cols() == 1) {
    if ((inequalities.array() < 0).any())
      throw std::runtime_error("Infeasible");
  } else if (inequalities.cols() == 2) {
    // Scalar inequalities, just find lower and upper bound
    double lower_bound = std::numeric_limits<double>::lowest(),
           upper_bound = std::numeric_limits<double>::max();
    for (int r = 0; r < inequalities.rows(); ++r) {
      double a = inequalities(r, 0), b = inequalities(r, 1);
      if (a > 0) {
        upper_bound = std::min(upper_bound, b / a);
      } else if (a < 0) {
        lower_bound = std::max(lower_bound, b / a);
      } else {
        // do nothing
      }
    }
    if (lower_bound > upper_bound) throw std::runtime_error("Infeasible");
    inequalities =
        (Matrix(2, 2) << 1, upper_bound, -1, -lower_bound).finished();
  } else {
    // This might grow exponentially so let's just ignore it
  }
}

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
