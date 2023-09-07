/**
 * @file utils.h
 * @brief Useful utilities
 * @author Gerry Chen
 * @date Sept 2023
 */

#pragma once

#include <unordered_map>

#include <gtsam/base/Testable.h>
#include <gtsam/base/VectorSpace.h>  // traits for double

#include "assert.h"

namespace gtsam {

/// Values type for the retiming problem, which is a scalar problem
using ScalarValues = std::unordered_map<Key, double>;

/// Testable for ScalarValues
template <>
struct traits<ScalarValues> {
  static void Print(const ScalarValues& values, const std::string& str = "") {
    std::cout << str << " {";
    for (const auto& [k, v] : values) {
      std::cout << DefaultKeyFormatter(k) << ":" << v << ", ";
    }
    std::cout << "}" << std::endl;
  }
  static bool Equals(const ScalarValues& m1, const ScalarValues& m2,
                     double tol = 1e-8) {
    return std::equal(m1.begin(), m1.end(), m2.begin(), m2.end(),
                      [tol](const auto& e1, const auto& e2) {
                        return e1.first == e2.first &&
                               traits<double>::Equals(e1.second, e2.second,
                                                      tol);
                      });
  }
};

/// Testable for std::vector
template <typename T>
struct traits<std::vector<T>> {
  static void Print(const std::vector<T>& v, const std::string& str = "") {
    std::cout << str << " {";
    for (const T& e : v) traits<T>::Print(e, ",");
    std::cout << "}" << std::endl;
  }
  static bool Equals(const std::vector<T>& m1, const std::vector<T>& m2,
                     double tol = 1e-8) {
    return std::equal(m1.begin(), m1.end(), m2.begin(), m2.end(),
                      [tol](const T& e1, const T& e2) {
                        return traits<T>::Equals(e1, e2, tol);
                      });
  }
};

/// Rekey for std::vector by calling rekey on each element
template <typename T>
void rekey_and_append(const std::vector<T>& src, const KeyVector& src_keys,
                      const KeyVector& dest_keys, std::vector<T>& dest) {
  dest.reserve(dest.size() + src.size());
  for (const T& e : src) dest.push_back(e.rekey(dest_keys, src_keys));
}
/// Rekey for std::vector by calling rekey on each element
template <typename T>
std::vector<T> rekey(const std::vector<T>& src, const KeyVector& src_keys,
                     const KeyVector& dest_keys) {
  std::vector<T> result;
  rekey_and_append(src, src_keys, dest_keys, result);
  return result;
}

/// Static Functions for Linear (In)Equality Constraints (Ax = b) or (Ax <= b)
struct LinearConstraint {
  using Linear = Eigen::Matrix<double, 1, Eigen::Dynamic>;
  using Linears = Matrix;

  /// Extract A, b from augmented matrix
  static Matrix getA(Matrix& src) { return src.leftCols(src.cols() - 1); }
  static Vector getb(Matrix& src) { return src.rightCols<1>(); }

  /// Rekey the linear constraint by transposing the columns
  static Matrix rekey(const Matrix& src, const KeyVector& src_keys,
                      const KeyVector& dest_keys) {
    Matrix result(src.rows(), dest_keys.size() + 1);
    result.setZero();

    // Use a linear search because keys should only be 2-3 elements long so
    // unordered_map probably slower
    for (int i = 0; i < src_keys.size(); ++i) {
      const auto& src_key = src_keys.at(i);
      const auto& dest_it =
          std::find(dest_keys.begin(), dest_keys.end(), src_key);
      if (dest_it != dest_keys.end()) {
        const auto& dest_loc = std::distance(dest_keys.begin(), dest_it);
        result.col(dest_loc) = src.col(i);
      }
    }
    result.rightCols<1>() = src.rightCols<1>();
    return result;
  }

  /// Gauss-Jordan Elimination of one column.  Returns a matrix (r)x(c-1)
  static Matrix eliminate(const Matrix& src, const Linear& row, const int col) {
    Matrix result(src.rows(), src.cols() - 1);
    // Set left-half
    if (col > 0) {
      result.leftCols(col - 1) =
          src.leftCols(col - 1) - src.col(col) / row(col) * row.head(col - 1);
    }
    // Set right-half
    if (col < src.cols() - 1) {
      result.rightCols(src.cols() - col - 1) =
          src.rightCols(src.cols() - col - 1) -
          src.col(col) / row(col) * row.tail(src.cols() - col - 1);
    }
    return result;
  }

  /// Erase one row of a matrix
  static Matrix dropRow(const Matrix& src, const int row) {
    Matrix result(src.rows() - 1, src.cols());
    result << src.topRows(row), src.bottomRows(src.rows() - row - 1);
    return result;
  }

  /// Erase one column of a matrix
  static Matrix dropCol(const Matrix& src, const int col) {
    Matrix result(src.rows(), src.cols() - 1);
    result << src.leftCols(col), src.rightCols(src.cols() - col - 1);
    return result;
  }

  // Print a single row (one linear constraint)
  static void print(const Vector& row, const std::string& s = "",
                    const std::string& equalityString = "=",
                    std::optional<KeyVector> keys = std::nullopt,
                    const KeyFormatter& formatter = DefaultKeyFormatter) {
    std::cout << s << " ";
    for (int i = 0; i < row.size() - 1; ++i) {
      if (keys) {
        std::cout << row(i) << "*" << formatter(keys->at(i));
      } else {
        std::cout << row(i) << ".x" << i;
      }
      if (i < row.size() - 2) std::cout << " + ";
    }
    std::cout << " " << equalityString << " " << row.tail(1) << std::endl;
  }
};

}  // namespace gtsam
