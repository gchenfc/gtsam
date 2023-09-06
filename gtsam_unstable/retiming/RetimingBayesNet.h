/**
 * @file RetimingBayesNet.h
 * @brief
 * @author Gerry Chen
 * @date Sept 2023
 */

#pragma once

#include <gtsam/inference/BayesNet.h>

#include "RetimingConditional.h"

namespace gtsam {

class GTSAM_EXPORT RetimingBayesNet : public BayesNet<RetimingConditional> {
 public:
  using Base = BayesNet<RetimingConditional>;
  using This = RetimingBayesNet;
  using ConditionalType = RetimingConditional;
  using shared_ptr = std::shared_ptr<This>;
  using sharedConditional = std::shared_ptr<ConditionalType>;
};

}  // namespace gtsam
