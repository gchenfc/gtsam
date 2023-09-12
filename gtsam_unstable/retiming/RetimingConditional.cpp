#include "RetimingConditional.h"

#include "eliminate.h"

using Linears = gtsam::LinearConstraint::Linears;
using Linear = gtsam::LinearConstraint::Linear;

namespace gtsam {

Linears substituteLinears(const Linears& Ab, const KeyVector& keys,
                          const ScalarValues& parents) {
  if (Ab.cols() == 2) return Ab;

  Linear y(Ab.cols() - 2);
  for (int c = 1; c < Ab.cols() - 1; ++c) {
    y(c - 1) = parents.at(keys.at(c));
  }

  Linears res(Ab.rows(), 2);
  const auto& rhs =
      Ab.rightCols<1>() - Ab.middleCols(1, Ab.cols() - 2) * y.transpose();
  res << Ab.leftCols<1>(), rhs;
  return res;
}

double solveEqualities(const Linears& equalities, const KeyVector& keys,
                       const ScalarValues& parents) {
  auto res = substituteLinears(equalities, keys, parents);
  RetimingFactor::removeRedundantEqualitiesInplace(res);
  return res(0, 1) / res(0, 0);
}

double solveInequalitiesGreedily(const Linears& inequalities,
                                 const KeyVector& keys,
                                 const ScalarValues& parents) {
  auto res = substituteLinears(inequalities, keys, parents);
  RetimingFactor::removeRedundantInequalitiesInplace(res);
  // Return the upper bound
  assertm(res(0, 0) == 1, "No upper bound on inequality");
  return res(0, 1);
}

double solveInequalityQuadratic1d(const PiecewiseQuadratic1d& objective,
                                  double xmin, double xmax) {
  const auto& xc = objective.xc;
  const auto& C = objective.C;
  auto start_index =
      std::distance(xc.begin(), std::upper_bound(xc.begin(), xc.end(), xmin));
  auto end_index =
      std::distance(xc.begin(), std::lower_bound(xc.begin(), xc.end(), xmax));
  auto len = end_index - start_index;

  if (xc.size() < C.rows()) {  // doesn't have book-ends
    PiecewiseQuadratic1d new_obj{
        .C = Eigen::Matrix<double, Eigen::Dynamic, 3>(len + 1, 3),
        .xc = Eigen::VectorXd(len + 2)};
    new_obj.C = C.middleRows(start_index, len + 1);
    new_obj.xc << xmin, xc.segment(start_index, len), xmax;
    return new_obj.argmin();
  } else {  // does have book-ends
    if (start_index == 0) {
      xmin = xc(0);
      start_index = 1;
    }
    if (end_index == xc.size()) {
      xmax = xc(xc.size() - 1);
      end_index = xc.size() - 1;
    }
    PiecewiseQuadratic1d new_obj{
        .C = Eigen::Matrix<double, Eigen::Dynamic, 3>(len + 1, 3),
        .xc = Eigen::VectorXd(len + 2)};
    new_obj.C = C.middleRows(start_index - 1, len + 1);
    new_obj.xc << xmin, xc.segment(start_index, len), xmax;
    return new_obj.argmin();
  }
}

double solveInequalityQuadratic(
    const PiecewiseQuadratic& objective,
    const PiecewiseQuadratic::Inequalities& inequalities, const KeyVector& keys,
    const ScalarValues& parents) {
  // traits<KeyVector>::Print(keys, "  Calling solveInequalityQuadratic on ");

  if (keys.size() == 1) {
    assertm(inequalities.cols() == 2, "1 variable should have 2 columns");
    double xmin = -std::numeric_limits<double>::infinity(),
           xmax = std::numeric_limits<double>::infinity();
    for (const auto& row : inequalities.rowwise()) {
      if (row(0) == 1) {
        xmax = std::min(xmax, row(1));
      } else if (row(0) == -1) {
        xmin = std::max(xmin, row(1));
      } else {
        throw std::runtime_error("Inequality should be 1 or -1");
      }
    }
    return solveInequalityQuadratic1d(objective.as1d(), xmin, xmax);
  }

  assertm(keys.size() == 2,
          "Quadratic inequality with more than 2 keys not implemented yet");
  assertm(inequalities.cols() == 3, "2 variables should have 3 columns");

  const auto &key1 = keys.front(), &key2 = keys.back();

  Eigen::Matrix<double, 1, 3> equality;
  PiecewiseQuadratic1d obj;

  // Extract the appropriate keys from parents
  int col;
  if (parents.find(key1) != parents.end()) {
    if (parents.find(key2) != parents.end()) {
      return objective.evaluate(parents.at(key1), parents.at(key2));
    } else {
      throw std::runtime_error("TODO");
      equality << 1, 0, parents.at(key1);
      obj = objective.substitute(parents.at(key1));
      col = 0;
      // auto i =
      //     std::distance(xc.begin(), std::upper_bound(xc.begin(), xc.end(),
      //     x));
      // if (xc.size() > C.rows()) --i;  // compensate for bookends
      // if (i < 0) throw std::runtime_error("x is out of bounds");
    }
  } else {
    if (parents.find(key2) != parents.end()) {
      equality << 0, 1, parents.at(key2);
      obj = objective.substitute(parents.at(key2));
      col = 1;
    } else {
      throw std::runtime_error(
          "Solving objective with 2 keys but neither is in parents");
    }
  }

  // Substitute parents `equality` and solve the 1d problem with x-bounds.
  auto bounds = LinearConstraint::eliminate(inequalities, equality, col);
  RetimingConditional::removeRedundantInequalitiesInplace(bounds);
  double xmin = -std::numeric_limits<double>::infinity(),
         xmax = std::numeric_limits<double>::infinity();
  for (const auto& row : bounds.rowwise()) {
    if (row(0) == 1) xmax = std::min(xmax, row(1));
    if (row(0) == -1) xmin = -std::max(xmin, row(1));
  }
  printf("  Bounds is (%f, %f)\n", xmin, xmax);
  return solveInequalityQuadratic1d(obj, xmin, xmax);
}

double solveUnconditionalQuadratic(const PiecewiseQuadratic& objective,
                                   const KeyVector& keys,
                                   const ScalarValues& parents) {
  if (keys.size() == 1) {
    const auto& obj = objective.as1d();
    const auto& key = keys.front();
    if (parents.find(key) != parents.end()) {
      return obj.evaluate(parents.at(key));
    } else {
      return obj.argmin();
    }
  } else if (keys.size() == 2) {
    const auto &key1 = keys.front(), &key2 = keys.back();
    if (parents.find(key1) != parents.end()) {
      if (parents.find(key2) != parents.end()) {
        return objective.evaluate(parents.at(key1), parents.at(key2));
      } else {
        return objective.substitute(parents.at(key1)).argmin();
      }
    } else {
      if (parents.find(key2) != parents.end()) {
        return objective.substitute(parents.at(key2)).argmin();
      } else {
        throw std::runtime_error(
            "Solving objective with 2 keys but neither is in parents");
      }
    }
  } else {
    throw std::runtime_error(
        "Solving objective with more than 2 keys not implemented yet");
  }
}

/******************************************************************************/

double RetimingConditional::solve(const ScalarValues& parents) {
  // this->print("Calling solve on ");
  traits<KeyVector>::Print(this->keys(), "Calling solve on");
  double result = [&]() -> double {
    if (equalities().size()) {
      return solveEqualities(equalities(), keys(), parents);
    } else if (inequalities().size()) {
      if (elimination_helpers::AllObjectivesGreedy(objectives())) {
        return solveInequalitiesGreedily(inequalities(), keys(), parents);
      } else {
        return solveInequalityQuadratic(PiecewiseQuadratic(objectives()),
                                        inequalities(), keys(), parents);
      }
    } else if (objectives().size()) {
      return solveUnconditionalQuadratic(PiecewiseQuadratic(objectives()),
                                         keys(), parents);
    } else {
      return -std::numeric_limits<double>::infinity();
    }
  }();
  std::cout << "  Returning " << result << std::endl;
  return result;
}

}  // namespace gtsam
