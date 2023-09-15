/**
 * @file CableRobotExample.cpp
 * @brief Loads parameters from data/CableRobotProblem.h
 * @author Gerry Chen
 * @date Sept 2023
 */

#include <gtsam/inference/Symbol.h>
#include <gtsam/base/timing.h>

#include <gtsam_unstable/retiming/RetimingFactorGraph.h>
#include <gtsam_unstable/retiming/RetimingBayesNet.h>
#include <gtsam_unstable/retiming/utils.h>

#include "data/CableRobotProblem.h"

using namespace gtsam;
using namespace gtsam::LinearConstraint;
using gtsam::symbol_shorthand::U;
using gtsam::symbol_shorthand::X;

Ordering ordering1(int N) {
  Ordering ordering;
  for (int i = 0; i < N; ++i) {
    ordering.push_back(U(i));
  }
  for (int i = 0; i <= N; ++i) {
    ordering.push_back(X(i));
  }
  return ordering;
}
Ordering ordering2(int N) {
  Ordering ordering;
  for (int i = 0; i < N; ++i) {
    ordering.push_back(U(i));
    ordering.push_back(X(i));
  }
  ordering.push_back(X(N));
  return ordering;
}
Ordering ordering3(int N) {
  Ordering ordering;
  for (int i = N - 1; i >= 0; --i) {
    ordering.push_back(U(i));
    ordering.push_back(X(i + 1));
  }
  ordering.push_back(X(0));
  return ordering;
}

template <size_t N, size_t M>
Eigen::Matrix<double, N, M> ToMatrix(
    const std::array<std::array<double, M>, N>& arr) {
  Eigen::Matrix<double, N, M> mat;
  for (int i = 0; i < N; ++i) {
    mat.row(i) = Eigen::Map<const Eigen::Matrix<double, 1, M>>(arr[i].data());
  }
  return mat;
}
template <size_t N>
Eigen::Matrix<double, 1, N> ToMatrix(const std::array<double, N>& arr) {
  return Eigen::Map<const Eigen::Matrix<double, 1, N>>(arr.data());
}

inline void RunSystem(bool with_objectives = true, bool print_sol = false) {
  tictoc_reset_();

  auto N = std::size(constr_x);
  double delta = 10.0 / N;
  std::cout << "Delta is: " << delta << std::endl;

  RetimingFactorGraph factors;

  factors.push_back(RetimingFactor::Equality({X(0)}, Create({1}, 0.0)));
  factors.push_back(RetimingFactor::Equality({X(N)}, Create({1}, 0.0)));

  for (int i = 0; i < N; ++i) {
    const auto& cx = ToMatrix(constr_x.at(i));
    const auto& cu = ToMatrix(constr_u.at(i));
    const auto& q = ToMatrix(quad_x.at(i));
    const auto& r = ToMatrix(quad_u.at(i));
    factors += RetimingFactor::Equality({X(i + 1), X(i), U(i)},
                                        Create({-1.0, 1.0, 2 * delta}, 0.0));
    factors += std::make_shared<RetimingFactor>(
        KeyVector{X(i)}, RetimingObjectives{}, Linears{}, cx);
    // factors += std::make_shared<RetimingFactor>(
    //     KeyVector{X(i)}, RetimingObjectives{}, Linears{},
    //     (Linears(2, 2) << 1, 10, -1, -0.01).finished());
    factors += std::make_shared<RetimingFactor>(
        KeyVector{U(i), X(i)}, RetimingObjectives{}, Linears{}, cu);
    if (with_objectives) {
      factors += RetimingFactor::Objective(
          {X(i)},
          PiecewiseQuadratic(PiecewiseQuadratic1d::Create({500 * q}, {})));
      factors += RetimingFactor::Objective({U(i), X(i)},
                                           PiecewiseQuadratic(1 * r, {}));
    }
  }

  // Do solving & Time
  for (int i = 0; i < 10; ++i) {
    gttic_(Solve_ordering3);
    auto bn = factors.eliminateSequential(ordering2(N));
    auto sol = bn->optimize();
  }

  auto bn = factors.eliminateSequential(ordering2(N));
  auto sol = bn->optimize();
  if (print_sol) {
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

  // Export to file
  {
    //
    std::ofstream file(with_objectives ? "cable_robot_qopp_solution.csv"
                                       : "cable_robot_topp_solution.csv");
    for (int i = 0; i < N; ++i) {
      file << sol.at(X(i)) << "," << sol.at(U(i)) << "\n";
    }
    file << sol.at(X(N)) << "," << 99999 << "\n";
  }

  tictoc_print_();
}

int main() {
  RunSystem(true, true);
  RunSystem(false, true);
  return 0;
}
