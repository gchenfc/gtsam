/**
 * @file RetimingFactor.h
 * @brief RetimingFactor is a factor that represents an objective, linear
 * equality constraint, and/or linear inequality constraint for a set of
 * scalar variables.
 * @author Gerry Chen
 * @date Sept 2023
 */
#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/VectorSpace.h>  // traits for double
#include <gtsam/inference/Factor.h>
#include <gtsam/linear/GaussianFactor.h>
#include "RetimingObjective.h"
#include "utils.h"

namespace gtsam {

class RetimingFactorGraph;

class GTSAM_EXPORT RetimingFactor : public Factor {
 public:
  using shared_ptr = std::shared_ptr<RetimingFactor>;
  using This = RetimingFactor;
  using Linear = LinearConstraint::Linear;
  using Linears = LinearConstraint::Linears;

  // Constructors
  RetimingFactor() : Factor() {}

  RetimingFactor(const KeyVector& keys, const RetimingObjectives& objectives,
                 const Linears& equality, const Linears& inequality,
                 bool removeRedundantConstraints = true,
                 bool checkForInfeasibility = true)
      : Factor(keys),
        objectives_(objectives),
        equalities_(equality),
        inequalities_(inequality) {
    if (removeRedundantConstraints) {
      removeRedundantEqualitiesInplace(equalities_, checkForInfeasibility);
      removeRedundantInequalitiesInplace(inequalities_);
    }
  }

  RetimingFactor(const RetimingFactorGraph& factors,
                 bool removeRedundantConstraints = true,
                 bool checkForInfeasibility = true);
  RetimingFactor(const RetimingFactorGraph& factors, const KeyVector& ordering,
                 bool removeRedundantConstraints = true,
                 bool checkForInfeasibility = true);

  // Static convenience constructors
  static shared_ptr Objective(const KeyVector& keys,
                              const RetimingObjective& objective) {
    return std::make_shared<RetimingFactor>(keys, RetimingObjectives{objective},
                                            Linears(0, keys.size() + 1),
                                            Linears(0, keys.size() + 1), false);
  }
  static shared_ptr Equality(const KeyVector& keys, const Linear& equality) {
    return std::make_shared<RetimingFactor>(keys, RetimingObjectives{},
                                            equality,
                                            Linears(0, keys.size() + 1), false);
  }
  static shared_ptr Inequality(const KeyVector& keys,
                               const Linear& inequality) {
    return std::make_shared<RetimingFactor>(keys, RetimingObjectives{},
                                            Linears(0, keys.size() + 1),
                                            inequality, false);
  }

  /// Substitute a variable with a linear equality constraint by performing
  /// Gaussian elimination on that column
  shared_ptr substitute(const size_t& column, const Linear& equality) const;

  /// Remove redundant equality constraints using QR
  static void removeRedundantEqualitiesInplace(
      Matrix& equalities, bool checkForInfeasibility = true);

  /// Remove redundant inequality constraints if only 1 variable is involved
  static void removeRedundantInequalitiesInplace(
      Matrix& inequalities, double infeasibilityTol = 1e-12);

  /// Normalize equality constraints by dividing by largest coefficients
  static void normalizeEqualitiesInplace(Matrix& equalities);

  /// Normalize inequality constraints by dividing by abs(largest coefficients)
  static void normalizeInequalitiesInplace(Matrix& inequalities);

  // Getters
  const RetimingObjectives& objectives() const { return objectives_; }
  const Linears& equalities() const { return equalities_; }
  const Linears& inequalities() const { return inequalities_; }

  RetimingObjectives& objectives() { return objectives_; }
  Linears& equalities() { return equalities_; }
  Linears& inequalities() { return inequalities_; }

  // Testable
  void print(
      const std::string& s = "Retiming Factor",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override;
  bool equals(const This& other, double tol = 1e-9) const;

 protected:
  // Member variables
  RetimingObjectives objectives_;
  Linears equalities_;
  Linears inequalities_;
};

template <>
struct traits<RetimingFactor> : public Testable<RetimingFactor> {};

}  // namespace gtsam
