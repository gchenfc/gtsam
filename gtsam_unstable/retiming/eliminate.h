/**
 * @file eliminate.h
 * @brief Elimination functions
 * @author Gerry Chen
 * @date Sept 2023
 */

#pragma once

#include "RetimingFactor.h"
#include "utils.h"

namespace gtsam {

// Forward declarations
class RetimingConditional;
class RetimingFactor;
class RetimingFactorGraph;
class Ordering;

/// Primary Elimination Function
GTSAM_EXPORT std::pair<std::shared_ptr<RetimingConditional>,
                       std::shared_ptr<RetimingFactor>>
EliminateRetiming(const RetimingFactorGraph& factors, const Ordering& keys);

/// Some helper stuff
using EqualityConditional = std::pair<double, ScalarValues>;  /// x = b + A.y

/// Solve an equation in terms of variable `key`.
std::optional<EqualityConditional> SolveEquality(const RetimingFactor& factor,
                                                 const Key& key);

}  // namespace gtsam
