/**
 * @file eliminate.h
 * @brief Elimination functions
 * @author Gerry Chen
 * @date Sept 2023
 */

#pragma once

#include "RetimingFactor.h"

namespace gtsam {

// Forward declarations
class RetimingConditional;
class RetimingFactor;
class RetimingFactorGraph;
class Ordering;

GTSAM_EXPORT std::pair<std::shared_ptr<RetimingConditional>,
                       std::shared_ptr<RetimingFactor> >
EliminateRetiming(const RetimingFactorGraph& factors, const Ordering& keys) {
  return {nullptr, nullptr};
}

}  // namespace gtsam
