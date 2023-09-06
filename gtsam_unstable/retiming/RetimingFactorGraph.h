/**
 * @file RetimingFactorGraph.h
 * @brief RetimingFactorGraph is a factor graph that stores a trajectory
 * retiming problem.  The trajectory retiming problem is a linear-chain of
 * scalar variables connected by binary or trinary factors.
 * @author Gerry Chen
 * @date Sept 2023
 */

#pragma once

#include <gtsam/inference/EliminateableFactorGraph.h>
#include <gtsam/inference/FactorGraph.h>
#include "eliminate.h"

namespace gtsam {

// Forward declarations
class RetimingFactorGraph;
class RetimingFactor;
class RetimingConditional;
class RetimingBayesNet;
class RetimingEliminationTree;  // We won't ever do multifrontal so we'll never
class RetimingBayesTree;        // actually need these
class RetimingJunctionTree;     // ...

/* ************************************************************************* */
template <>
struct EliminationTraits<RetimingFactorGraph> {
  typedef RetimingFactor FactorType;
  typedef RetimingFactorGraph FactorGraphType;
  typedef RetimingConditional ConditionalType;
  typedef RetimingBayesNet BayesNetType;
  typedef RetimingEliminationTree EliminationTreeType;
  typedef RetimingBayesTree BayesTreeType;
  typedef RetimingJunctionTree JunctionTreeType;

  /// The default dense elimination function
  static std::pair<std::shared_ptr<ConditionalType>,
                   std::shared_ptr<FactorType>>
  DefaultEliminate(const FactorGraphType& factors, const Ordering& keys) {
    return EliminateRetiming(factors, keys);
  }
  /// The default ordering generation function
  static Ordering DefaultOrderingFunc(
      const FactorGraphType& graph,
      std::optional<std::reference_wrapper<const VariableIndex>>
          variableIndex) {
    return Ordering::Colamd((*variableIndex).get());
  }
};

/* ************************************************************************* */
/**
 * A Linear Factor Graph is a factor graph where all factors are Retiming, i.e.
 *   Factor == RetimingFactor
 *   VectorValues = A values structure of vectors
 * Most of the time, linear factor graphs arise by linearizing a non-linear
 * factor graph.
 */
class GTSAM_EXPORT RetimingFactorGraph
    : public FactorGraph<RetimingFactor>,
      public EliminateableFactorGraph<RetimingFactorGraph> {
 public:
  typedef RetimingFactorGraph This;
  typedef FactorGraph<RetimingFactor> Base;
  typedef EliminateableFactorGraph<This> BaseEliminateable;
  typedef std::shared_ptr<This> shared_ptr;

  /// @name Constructors
  /// @{

  /** Default constructor */
  RetimingFactorGraph() {}

  /// @}
  /// @name Testable
  /// @{

  bool equals(const This& fg, double tol = 1e-9) const;

  /// @}

  /// Check exact equality.
  friend bool operator==(const RetimingFactorGraph& lhs,
                         const RetimingFactorGraph& rhs) {
    return lhs.isEqual(rhs);
  }
};

}  // namespace gtsam
