/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testEliminateRetiming.cpp
 *  @brief  Unit tests for retiming/eliminate.h
 *  @author Gerry Chen
 **/

#include <gtsam_unstable/retiming/eliminate.h>

#include <gtsam_unstable/retiming/RetimingFactorGraph.h>
#include <gtsam_unstable/retiming/RetimingFactor.h>
#include <gtsam_unstable/retiming/RetimingConditional.h>
#include <gtsam_unstable/retiming/RetimingBayesNet.h>

using namespace std;
using namespace gtsam;
using gtsam::symbol_shorthand::U;
using gtsam::symbol_shorthand::X;

static constexpr int N = 100;

Matrix LinConstr(std::initializer_list<double> a, double b) {
  Vector ret(a.size() + 1);
  std::copy(a.begin(), a.end(), ret.begin());
  ret(a.size()) = b;
  return ret.transpose();
}

// Print Solution nicely
auto printSol = [](const ScalarValues& sol) {
  for (int i = 0; i < N; ++i) {
    std::cout << "\tx" << i << " = " << sol.at(X(i)) << ", u" << i << " = "
              << sol.at(U(i)) << "\n";
  }
  std::cout << "\txN = " << sol.at(X(N)) << "\n";
};

// Print Solution computer readable
auto exportSol = [](const ScalarValues& sol, const std::string& fname) {
  std::ofstream outfile(fname);
  for (int i = 0; i < N; ++i) {
    outfile << sol.at(X(i)) << "," << sol.at(U(i)) << "\n";
  }
  outfile << sol.at(X(N)) << ",", (1 / 0), "\n";
  outfile.close();
};

// Piecewise Linear Upper Limit for toy problem
double upper_lim(double t) {
  double x = [](double t) -> double {
    if (t < 0.2) return 1.0;
    if (t < 0.4) return 1 - 2 * (t - 0.2);
    if (t < 0.45) return 0.6 + 6 * (t - 0.4);
    if (t < 0.8) return 0.9 - 10 / 7. * (t - 0.45);
    return 0.4 + 0.6 / 0.2 * (t - 0.8);
  }(t);
  return x * x;
};

Ordering createOrdering() {
  Ordering ordering;
  for (int i = 0; i < N; ++i) {
    ordering.push_back(U(i));
  }
  for (int i = N; i >= 0; --i) {
    ordering.push_back(X(i));
  }
  return ordering;
}

int main() {
  RetimingFactorGraph factors;
  factors.push_back(RetimingFactor::Equality({X(0)}, LinConstr({1}, 0)));
  factors.push_back(RetimingFactor::Equality({X(N)}, LinConstr({1}, 0)));
  static constexpr double umax = 0.4;
  for (int i = 0; i < N; ++i) {
    factors.push_back(RetimingFactor::Equality(
        {X(i + 1), X(i), U(i)}, LinConstr({-1.0, 1.0, 0.1}, 0.0)));
    factors.push_back(RetimingFactor::Inequality({U(i)}, LinConstr({1}, umax)));
    factors.push_back(RetimingFactor::Inequality({U(i)}, LinConstr({-1}, umax)));
  }
  std::ofstream outfile("prob.csv");
  for (int i = 0; i <= N; ++i) {
    factors.push_back(RetimingFactor::Inequality(
        {X(i)}, LinConstr({1}, upper_lim((double)i / N))));
    outfile << upper_lim((double)i / N) << "," << umax << "\n";
  }
  outfile.close();
  auto ordering = createOrdering();

  /******************** SOLVE ******************/
  {
    gttic_(TOPP);
    auto bn = factors.eliminateSequential(ordering);
    auto sol = bn->optimize();
    gttoc_(TOPP);
    exportSol(sol, "topp.csv");
  }

  // Add quadratic objectives
  for (int i = 0; i < N; ++i) {
    // factors.push_back(RetimingFactor::Objective(
    //     {X(i)}, PiecewiseQuadratic(1, 0, 0, -1.8, 0, 0.81)));
    factors.push_back(RetimingFactor::Objective(
        {X(i)}, PiecewiseQuadratic(1, 0, 0, -1.6, 0, 0.81)));
    factors.push_back(RetimingFactor::Objective(
        {U(i)}, PiecewiseQuadratic(3, 0, 0, 0, 0, 0)));
  }

  // Solve again
  {
    gttic_(QOPP);
    auto bn = factors.eliminateSequential(ordering);
    auto sol = bn->optimize();
    gttoc_(QOPP);
    exportSol(sol, "qopp.csv");
    printSol(sol);
  }

  tictoc_print_();
}
