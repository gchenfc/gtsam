#include "RetimingFactor.h"

#include "RetimingFactorGraph.h"

namespace gtsam {

RetimingFactor::RetimingFactor(const RetimingFactorGraph& factors,
                               bool removeRedundantConstraints,
                               bool checkForInfeasibility)
    : RetimingFactor{factors, factors.keyVector(), removeRedundantConstraints,
                     checkForInfeasibility} {}

RetimingFactor::RetimingFactor(const RetimingFactorGraph& factors,
                               const KeyVector& ordering,
                               bool removeRedundantConstraints,
                               bool checkForInfeasibility)
    : RetimingFactor{ordering,
                     factors.objectives(ordering),
                     factors.equalities(ordering),
                     factors.inequalities(ordering),
                     removeRedundantConstraints,
                     checkForInfeasibility} {}

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
        assertm(abs(Ab(r, Ab.cols() - 1)) < 1e-12,
                "Infeasible due to equality");
      } else {
        break;  // No more zero rows
      }
    }
  }
}

/******************************************************************************/

void RetimingFactor::removeRedundantInequalitiesInplace(
    Matrix& inequalities, double infeasibilityTol) {
  if (inequalities.cols() == 1) {
    assertm((inequalities.array() >= 0).all(), "Infeasible due to inequality");
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
    assertm(lower_bound <= upper_bound + infeasibilityTol,
            "Infeasible due to inequalities");

    bool has_lower = (lower_bound != std::numeric_limits<double>::lowest()),
         has_upper = (upper_bound != std::numeric_limits<double>::max());
    if (has_lower && has_upper) {
      inequalities =
          (Matrix(2, 2) << 1, upper_bound, -1, -lower_bound).finished();
    } else if (!has_lower && has_upper) {
      inequalities = (Matrix(1, 2) << 1, upper_bound).finished();
    } else if (has_lower && !has_upper) {
      inequalities = (Matrix(1, 2) << -1, -lower_bound).finished();
    } else {
      inequalities = Matrix(0, 2);
    }

  } else {
    // This might grow exponentially so let's just ignore it
  }
}

/******************************************************************************/

void RetimingFactor::normalizeEqualitiesInplace(Matrix& equalities) {
  for (auto row : equalities.rowwise()) {  // rowwise gives mutable refs
    const double& amt = row.head(row.size() - 1).maxCoeff();
    row /= amt;
  }
}

/******************************************************************************/

void RetimingFactor::normalizeInequalitiesInplace(Matrix& inequalities) {
  for (auto row : inequalities.rowwise()) {  // rowwise gives mutable refs
    const double& amt = row.head(row.size() - 1).maxCoeff();
    row /= abs(amt);
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

/******************************************************************************/

void RetimingFactor::print(const std::string& s,
                           const KeyFormatter& formatter) const {
  this->Factor::print(s, formatter);
  for (auto o : objectives_) {
    traits<RetimingObjective>::Print(o, "\tCost:");
  }
  for (auto e : equalities_.rowwise()) {
    LinearConstraint::print(e, "\tEquality Constraint:", "=", keys_);
  }
  for (auto e : inequalities_.rowwise()) {
    LinearConstraint::print(e, "\tInequality Constraint:", "<=", keys_);
  }
}

/******************************************************************************/

bool RetimingFactor::equals(const This& other, double tol) const {
  return traits<RetimingObjectives>::Equals(objectives_, other.objectives_,
                                            tol) &&
         traits<Linears>::Equals(equalities_, other.equalities_, tol) &&
         traits<Linears>::Equals(inequalities_, other.inequalities_, tol);
}

}  // namespace gtsam
