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

inline void RunSystem(int N = 100000, bool use_objectives = false,
               bool print_sol = false) {
  tictoc_reset_();

  RetimingFactorGraph factors;
  for (int i = 0; i < N; ++i) {
    if (use_objectives) {
      factors.push_back(RetimingFactor::Objective(
          {X(i), U(i)}, PiecewiseQuadratic(1, 1, 0, 0, 0, 0)));
    }
    factors.push_back(RetimingFactor::Equality({X(i + 1), X(i), U(i)},
                                               Create({-1.0, 1.0, 0.5}, 0.0)));
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
  if (!use_objectives) {
    for (int i = 0; i < 10; ++i) {
      gttic_(Solve_COLAMD);
      auto bn = factors.eliminateSequential();  // COLAMD
      auto sol = bn->optimize();
    }
  }
  for (int i = 0; i < 10; ++i) {
    gttic_(Solve_ordering1);
    auto bn1 = factors.eliminateSequential(ordering1);
    auto sol1 = bn1->optimize();
  }
  for (int i = 0; i < 10; ++i) {
    gttic_(Solve_ordering2);
    auto bn2 = factors.eliminateSequential(ordering2);
    auto sol2 = bn2->optimize();
  }
  for (int i = 0; i < 10; ++i) {
    gttic_(Solve_ordering3);
    auto bn2 = factors.eliminateSequential(ordering3);
    auto sol2 = bn2->optimize();
  }

  if (print_sol) {
    auto bn = factors.eliminateSequential(ordering3);
    auto sol = bn->optimize();

    // Print Solution nicely
    auto printSol = [&N](const ScalarValues& sol) {
      for (int i = 0; i < N; ++i) {
        std::cout << "\tx" << i << " = " << sol.at(X(i)) << ", u" << i << " = "
                  << sol.at(U(i)) << "\n";
      }
      std::cout << "\txN = " << sol.at(X(N)) << "\n";
    };
    // bn->print("Bayes Net:");
    printSol(sol);
  }
  tictoc_print_();
}

int main() {
  for (int N = 1000; N <= 50000; N += 1000) {
    std::cout << "Running with N = " << N << std::endl;
    RunSystem(N);
  }
  // for (int N = 1000; N <= 10000; N += 1000) {
  //   std::cout << "Running with N = " << N << std::endl;
  //   RunSystem(N, false);
  //   break;
  // }
  return 0;
}
