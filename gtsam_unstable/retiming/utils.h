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

/// Values type for the retiming problem, which is a scalar problem
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

namespace retiming {
/// Linear (In)Equality Constraint (Ax = b) or (Ax <= b)
struct Linear {
  std::vector<double> A;
  double b;

  /// Rekey the linear constraint by shifting the coefficients to new positions
  Linear rekey(const KeyVector& dest_keys, const KeyVector& src_keys) const {
    Linear result;
    result.A.resize(dest_keys.size(), 0.0);
    // Use a linear search because keys should only be 2-3 elements long so
    // unordered_map probably slower
    for (int i = 0; i < src_keys.size(); ++i) {
      const auto& src_key = src_keys.at(i);
      const auto& dest_it =
          std::find(dest_keys.begin(), dest_keys.end(), src_key);
      if (dest_it != dest_keys.end()) {
        const auto& dest_loc = std::distance(dest_keys.begin(), dest_it);
        result.A.at(dest_loc) = A.at(i);
      }
    }
    result.b = b;
    return result;
  }

  // Testable
  void print(const std::string& s = "Linear",
             const KeyFormatter& formatter = DefaultKeyFormatter) const {
    std::cout << s << " ";
    for (int i = 0; i < A.size(); ++i) {
      std::cout << A.at(i) << ".x" << i;
      if (i < A.size() - 1) std::cout << " + ";
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
