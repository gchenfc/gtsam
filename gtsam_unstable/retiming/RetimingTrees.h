/**
 * @file RetimingTrees.h
 * @brief A bunch of boilerplate code needed for variable elimination in GTSAM
 * @author Gerry Chen
 * @date Sept 2023
 */

#pragma once

#include <gtsam/inference/Key.h>
#include <gtsam/inference/EliminationTree.h>
#include <gtsam/inference/BayesTreeCliqueBase.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/inference/JunctionTree.h>

#include "RetimingBayesNet.h"
#include "RetimingFactorGraph.h"

namespace gtsam {

/***************************** Elimination Tree *****************************/
class RetimingEliminationTree
    : public EliminationTree<RetimingBayesNet, RetimingFactorGraph> {
 public:
  using Base = EliminationTree<RetimingBayesNet, RetimingFactorGraph>;
  using This = RetimingEliminationTree;
  using shared_ptr = std::shared_ptr<This>;

  RetimingEliminationTree(const RetimingFactorGraph& factorGraph,
                          const VariableIndex& structure,
                          const Ordering& order);

  RetimingEliminationTree(const RetimingFactorGraph& factorGraph,
                          const Ordering& order);

  bool equals(const This& other, double tol = 1e-9) const;
};

/***************************** Bayes Tree *****************************/
class RetimingBayesTreeClique
    : public BayesTreeCliqueBase<RetimingBayesTreeClique, RetimingFactorGraph> {
 public:
  using This = RetimingBayesTreeClique;
  using Base =
      BayesTreeCliqueBase<RetimingBayesTreeClique, RetimingFactorGraph>;
  using shared_ptr = std::shared_ptr<This>;
  using weak_ptr = std::weak_ptr<This>;
  RetimingBayesTreeClique() {}
  RetimingBayesTreeClique(
      const std::shared_ptr<RetimingConditional>& conditional);
};
class RetimingBayesTree : public BayesTree<RetimingBayesTreeClique> {};

/***************************** Junction Tree *****************************/
class RetimingJunctionTree
    : public JunctionTree<RetimingBayesTree, RetimingFactorGraph> {
 public:
  using Base = JunctionTree<RetimingBayesTree, RetimingFactorGraph>;
  using This = RetimingJunctionTree;
  using shared_ptr = std::shared_ptr<This>;

  RetimingJunctionTree(const RetimingEliminationTree& eliminationTree);
};

}  // namespace gtsam
