/**
 * @file RetimingBayesNet.h
 * @brief
 * @author Gerry Chen
 * @date Sept 2023
 */

#pragma once

#include <gtsam/inference/BayesNet.h>

#include "RetimingConditional.h"
#include "utils.h"

namespace gtsam {

class GTSAM_EXPORT RetimingBayesNet : public BayesNet<RetimingConditional> {
 public:
  using Base = BayesNet<RetimingConditional>;
  using This = RetimingBayesNet;
  using ConditionalType = RetimingConditional;
  using shared_ptr = std::shared_ptr<This>;
  using sharedConditional = std::shared_ptr<ConditionalType>;

  /// Compute the optimal variable assignments by doing a backward pass through
  /// the Bayes Net
  ScalarValues optimize() const { return optimize(ScalarValues()); }

  /// Compute the optimal variable assignments by doing a backward pass through
  /// the Bayes Net
  ScalarValues optimize(const ScalarValues& parentValues) const {
    ScalarValues solution = parentValues;
    // solve each node in reverse topological sort order (parents first)
    for (auto it = std::make_reverse_iterator(end());
         it != std::make_reverse_iterator(begin()); ++it) {
      assertm((solution.emplace((*it)->front(), (*it)->solve(solution)).second),
              "Key already exists in solution");
    }
    return solution;
  }
};

}  // namespace gtsam
