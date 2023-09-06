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

namespace gtsam {

// Values type for the retiming problem, which is a scalar problem
using ScalarValues = std::unordered_map<Key, double>;

/// Testable for std::vector
template <typename T>
struct traits<std::vector<T>> {
  static void Print(const std::vector<T>& v, const std::string& str = "") {
    std::cout << str << " {";
    for (const T& e : v) traits<T>::Print(v, ",");
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

namespace retiming {
/// Linear (In)Equality Constraint (Ax = b) or (Ax <= b)
struct Linear {
  std::vector<double> A;
  double b;
  // Testable
  void print(const std::string& s = "Linear",
             const KeyFormatter& formatter = DefaultKeyFormatter) const {
    std::cout << s << " ";
    for (int i = 0; i < A.size(); ++i) {
      std::cout << A.at(i) << ".x" << i;
      if (i < A.size() - 1) std::cout << "+";
    }
    std::cout << " = " << b << std::endl;
  }
  bool equals(const Linear& other, double tol = 1e-9) const {
    return traits<double>::Equals(b, other.b, tol) &&
           traits<std::vector<double>>::Equals(A, other.A, tol);
  }
};

}  // namespace retiming

template <>
struct traits<retiming::Linear> : public Testable<retiming::Linear> {};

}  // namespace gtsam
