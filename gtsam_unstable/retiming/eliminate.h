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

namespace elimination_helpers {

// Returns true if there are no non-greedy objectives
bool AllObjectivesGreedy(const RetimingObjectives& objectives);

/// Helper Elimination Function for special case of exactly 2 variables
GTSAM_EXPORT std::pair<std::shared_ptr<RetimingConditional>,
                       std::shared_ptr<RetimingFactor>>
EliminateLp2d(const RetimingFactor::shared_ptr& factor, const KeyVector& keys);

/// Helper Elimination Function for special case of >2 variables, but only 2
/// have inequalities.  This is an edge-case when the graph isn't smart enough
/// to realize the sparsity pattern despite the 0s
GTSAM_EXPORT std::pair<std::shared_ptr<RetimingConditional>,
                       std::shared_ptr<RetimingFactor>>
EliminateManyVars2Inequalities(const RetimingFactor& factor, KeyVector& keys);

/// Helper Elimination Function for 2 variables with inequalities and piecewise
/// quadratic objective
GTSAM_EXPORT std::pair<std::shared_ptr<RetimingConditional>,
                       std::shared_ptr<RetimingFactor>>
EliminateQp2d(const RetimingFactor::shared_ptr& factor, KeyVector& keys);

}  // namespace elimination_helpers

}  // namespace gtsam
