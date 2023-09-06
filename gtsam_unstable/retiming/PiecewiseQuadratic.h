/**
 * @file PiecewiseQuadratic.h
 * @brief A piecewise quadratic used to define objectives.
 * @author Gerry Chen
 * @date Sept 2023
 */

#pragma once

#include <memory>

namespace gtsam {

class PiecewiseQuadratic {
 public:
  using shared_ptr = std::shared_ptr<PiecewiseQuadratic>;
  using This = PiecewiseQuadratic;

  // TODO(gerry): implementation

  PiecewiseQuadratic rekey(const KeyVector& src_keys,
                           const KeyVector& dest_keys) const {
    std::cout << "Warning: Piecewise Quadratic not yet implemented"
              << std::endl;
    return *this;
  }

  // Testable
  void print(const std::string& s = "Piecewise Quadratic",
             const KeyFormatter& formatter = DefaultKeyFormatter) const {
    std::cout << s << " unimplemented\n";
  }
  bool equals(const This& other, double tol = 1e-9) const {
    std::cout << "Warning: Piecewise Quadratic not yet implemented"
              << std::endl;
    return true;
  }
};

template <>
struct traits<PiecewiseQuadratic> : public Testable<PiecewiseQuadratic> {};

}  // namespace gtsam
