/**
 * @file TimingTests.cpp
 * @brief This file runs some simple systems for timing.
 * @author Gerry Chen
 * @date Sept 2023
 */

#include <gtsam/inference/Symbol.h>
#include <gtsam/base/timing.h>

#include <gtsam_unstable/retiming/RetimingFactorGraph.h>
#include <gtsam_unstable/retiming/RetimingBayesNet.h>
#include <gtsam_unstable/retiming/utils.h>

using namespace gtsam;
using namespace gtsam::LinearConstraint;
using gtsam::symbol_shorthand::U;
using gtsam::symbol_shorthand::X;

int total_inequalities = 0;
int total_quadratic_segments = 0;
GTSAM_EXPORT std::pair<std::shared_ptr<RetimingConditional>,
                       std::shared_ptr<RetimingFactor>>
EliminateRetimingPrint(const RetimingFactorGraph& factors,
                       const Ordering& keys) {
  auto [cond, joint] = EliminateRetiming(factors, keys);
  if (joint) {
    // joint->print("Joint");
    for (auto key : joint->keys()) std::cout << DefaultKeyFormatter(key) << " ";
    assertm(joint->objectives().size() == 1, "Only 1 objective expected");
    std::cout << "," << joint->objectives()[0].quadratic.C().rows() << std::endl;
  }

  return {cond, joint};
}

void RunSystem(int N = 100000, bool use_objectives = false) {
  tictoc_reset_();

  RetimingFactorGraph factors;
  for (int i = 0; i < N; ++i) {
    if (use_objectives) {
      factors.push_back(RetimingFactor::Objective(
          {X(i), U(i)}, PiecewiseQuadratic(1, 1, 0, 0, 0, 0)));
    }
    factors.push_back(RetimingFactor::Equality({X(i + 1), X(i), U(i)},
                                               Create({-1.0, 1.01, 0.5}, 0.0)));
    factors.push_back(
        RetimingFactor::Inequality({U(i), X(i)}, Create({1, 1}, 0.1)));
  }
  if (use_objectives) {
    factors.push_back(RetimingFactor::Objective(
        {X(N)}, PiecewiseQuadratic(1, 0, 0, 0, 0, 0)));
  }
  factors.push_back(RetimingFactor::Equality({X(0)}, Create({1}, 1.0)));

  // Define orderings to use
  Ordering ordering1;
  for (int i = 0; i < N; ++i) {
    ordering1.push_back(U(i));
  }
  for (int i = 0; i <= N; ++i) {
    ordering1.push_back(X(i));
  }
  Ordering ordering2;
  for (int i = 0; i < N; ++i) {
    ordering2.push_back(U(i));
    ordering2.push_back(X(i));
  }
  ordering2.push_back(X(N));
  Ordering ordering3;
  for (int i = N - 1; i >= 0; --i) {
    ordering3.push_back(U(i));
    ordering3.push_back(X(i + 1));
  }
  ordering3.push_back(X(0));

  // Do solving & Time
  {
    std::cout << "Eliminate Ordering1" << std::endl;
    auto bn1 = factors.eliminateSequential(ordering1, EliminateRetimingPrint);
    auto sol1 = bn1->optimize();
  }
  // {
  //   std::cout << "Eliminate Ordering2" << std::endl;
  //   auto bn2 = factors.eliminateSequential(ordering2, EliminateRetimingPrint);
  //   auto sol2 = bn2->optimize();
  // }
  {
    std::cout << "Eliminate Ordering3" << std::endl;
    auto bn2 = factors.eliminateSequential(ordering3, EliminateRetimingPrint);
    auto sol2 = bn2->optimize();
  }
  // auto bn2 = factors.eliminateSequential(ordering3);
  // auto sol2 = bn2->optimize();

  // // Print Solution nicely
  // auto printSol = [&N](const ScalarValues& sol) {
  //   for (int i = 0; i < N; ++i) {
  //     std::cout << "\tx" << i << " = " << sol.at(X(i)) << ", u" << i << " = "
  //               << sol.at(U(i)) << "\n";
  //   }
  //   std::cout << "\txN = " << sol.at(X(N)) << "\n";
  // };
  // bn->print("Bayes Net:");
  // traits<ScalarValues>::Print(sol, "SOLUTION:");
  // printSol(sol);
  // bn1->print("Bayes Net:");
  // traits<ScalarValues>::Print(sol1, "SOLUTION:");
  // printSol(sol2);
}

int main() {
  // for (int N = 1000; N <= 10000; N += 1000) {
  //   std::cout << "Running with N = " << N << std::endl;
  //   RunSystem(N);
  // }
  for (int N = 200; N <= 10000; N += 1000) {
    std::cout << "Running with N = " << N << std::endl;
    RunSystem(N, true);
    return 0;
  }
  return 0;
}
