/**
 * @file RetimingTrees.cpp
 * @brief Instantiations / Definitions for RetimingTrees.h
 * @author Gerry Chen
 * @date Sept 2023
 */

#include "RetimingTrees.h"

#include <gtsam/inference/EliminationTree-inst.h>
#include <gtsam/inference/BayesTree-inst.h>
#include <gtsam/inference/BayesTreeCliqueBase-inst.h>
#include <gtsam/inference/JunctionTree-inst.h>

namespace gtsam {

// Instantiate EliminationTree base class
template class EliminationTree<RetimingBayesNet, RetimingFactorGraph>;

// Instantiate BayesTree base classes
template class BayesTreeCliqueBase<RetimingBayesTreeClique,
                                   RetimingFactorGraph>;
template class BayesTree<RetimingBayesTreeClique>;

// Instantiate JunctionTree base classes
template class EliminatableClusterTree<RetimingBayesTree, RetimingFactorGraph>;
template class JunctionTree<RetimingBayesTree, RetimingFactorGraph>;

/* Now define all the constructors and function implementations that forward to
 * the base classes.
 */
/***************************** Elimination Tree *****************************/
RetimingEliminationTree::RetimingEliminationTree(
    const RetimingFactorGraph& factorGraph, const VariableIndex& structure,
    const Ordering& order)
    : Base(factorGraph, structure, order) {}
RetimingEliminationTree::RetimingEliminationTree(
    const RetimingFactorGraph& factorGraph, const Ordering& order)
    : Base(factorGraph, order) {}
bool RetimingEliminationTree::equals(const This& other, double tol) const {
  return Base::equals(other, tol);
}

/***************************** Bayes Tree *****************************/
RetimingBayesTreeClique::RetimingBayesTreeClique(
    const std::shared_ptr<RetimingConditional>& conditional)
    : Base(conditional) {}

/***************************** Junction Tree *****************************/

RetimingJunctionTree::RetimingJunctionTree(
    const RetimingEliminationTree& eliminationTree)
    : Base(eliminationTree) {}

}  // namespace gtsam
