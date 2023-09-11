/**
 * @file Simple1dSystemExample.cpp
 * @brief This file runs a simple example of a 1d system with inequality
 * constraints.
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

void EXPECT_DOUBLES_EQUAL(double a, double b, double tol) {
  if (std::abs(a - b) > tol) {
    std::cout << "Expected " << a << " but got " << b << std::endl;
  }
}

void RunSystem(int N = 100000) {
  tictoc_reset_();
  // System Dynamics:
  //    x_{k+1} = 0.98 * x_k + 0.5 * u_k
  //    x_0 = 1.0
  //    x_1 <= 0.3
  //    u_k + x_k <= 0.1
  //
  // Expect solution to be:
  //    x0 = 1.0, u0 = -1.36  // So that x1 = 0.3
  //    x1 = 0.3, u1 = -0.2
  //    x2 = 0.194, u2 = -0.094
  //    x3 = 0.14312, u3 = -0.04312
  //    ...
  //    xinf = 0.1 / 1.04 = 0.09615384615, uinf = -0.00384615385

  RetimingFactorGraph factors;
  for (int i = 0; i < N; ++i) {
    factors.push_back(RetimingFactor::Equality({X(i + 1), X(i), U(i)},
                                               Create({-1.0, 0.98, 0.5}, 0.0)));
    factors.push_back(
        RetimingFactor::Inequality({U(i), X(i)}, Create({1, 1}, 0.1)));
  }
  factors.push_back(RetimingFactor::Equality({X(0)}, Create({1}, 1.0)));
  factors.push_back(RetimingFactor::Inequality({X(1)}, Create({1}, 0.3)));

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

  // Do solving & Time
  for (int i = 0; i < 10; ++i) {
    gttic_(Solve_COLAMD);
    auto bn = factors.eliminateSequential();  // COLAMD
    assert(bn);
    auto sol = bn->optimize();
  }
  for (int i = 0; i < 10; ++i) {
    gttic_(Solve_ordering1);
    auto bn1 = factors.eliminateSequential(ordering1);
    assert(bn1);
    auto sol1 = bn1->optimize();
  }
  for (int i = 0; i < 10; ++i) {
    gttic_(Solve_ordering2);
    auto bn2 = factors.eliminateSequential(ordering2);
    assert(bn2);
    auto sol2 = bn2->optimize();
  }

  // Print Solution nicely
  // auto printSol = [](const ScalarValues& sol) {
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
  // printSol(sol1);
  tictoc_print_();
}

int main() {
  for (int N = 1000; N <= 10000; N += 1000) {
    std::cout << "Running with N = " << N << std::endl;
    RunSystem(N);
  }
  return 0;
}
