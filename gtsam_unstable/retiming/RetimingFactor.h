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
  RetimingFactor(const KeyVector& keys, const RetimingObjectives& objectives,
                 const Linears& equality, const Linears& inequality)
      : Factor(keys),
        objectives_(objectives),
        equalities_(equality),
        inequalities_(inequality) {}

  RetimingFactor(const RetimingFactorGraph& factors);
  RetimingFactor(const RetimingFactorGraph& factors, const KeyVector& ordering);

  // Static convenience constructors
  static shared_ptr Objective(const KeyVector& keys,
                              const RetimingObjective& objective) {
    return std::make_shared<RetimingFactor>(keys, RetimingObjectives{objective},
                                            Linears(0, keys.size() + 1),
                                            Linears(0, keys.size() + 1));
  }
  static shared_ptr Equality(const KeyVector& keys, const Linear& equality) {
    return std::make_shared<RetimingFactor>(
        keys, RetimingObjectives{}, equality, Linears(0, keys.size() + 1));
  }
  static shared_ptr Inequality(const KeyVector& keys,
                               const Linear& inequality) {
    return std::make_shared<RetimingFactor>(
        keys, RetimingObjectives{}, Linears(0, keys.size() + 1), inequality);
  }

  /// Substitute a variable with a linear equality constraint by performing
  /// Gaussian elimination on that column
  shared_ptr substitute(const size_t& column, const Linear& equality) const;

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
      const KeyFormatter& formatter = DefaultKeyFormatter) const override {
    this->Factor::print(s, formatter);
    for (auto o : objectives_) {
      traits<RetimingObjective>::Print(o, "\tCost:");
    }
    for (auto e : equalities_.rowwise()) {
      LinearConstraint::print(e, "\tEquality Constraint:");
    }
    for (auto e : inequalities_.rowwise()) {
      LinearConstraint::print(e, "\tInequality Constraint:");
    }
  }
  bool equals(const This& other, double tol = 1e-9) const {
    return traits<RetimingObjectives>::Equals(objectives_, other.objectives_,
                                              tol) &&
           traits<Linears>::Equals(equalities_, other.equalities_, tol) &&
           traits<Linears>::Equals(inequalities_, other.inequalities_, tol);
  }

 protected:
  // Member variables
  RetimingObjectives objectives_;
  Linears equalities_;
  Linears inequalities_;
};

template <>
struct traits<RetimingFactor> : public Testable<RetimingFactor> {};

}  // namespace gtsam
