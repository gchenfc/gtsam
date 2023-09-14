/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testRemoveRepeats.cpp
 *  @brief  Unit tests for removing repeats
 *  @author Gerry Chen
 **/

#include <gtsam_unstable/retiming/removeRepeats.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

using namespace std;
using namespace gtsam;

bool equals_print(const std::vector<double>& exp,
                  const std::vector<double>& act) {
  bool ret = std::equal(exp.begin(), exp.end(), act.begin(), act.end(),
                        [](double a, double b) { return fabs(a - b) < 1e-8; });
  if (!ret) {
    std::cout << "Expected: ";
    for (const auto& e : exp) std::cout << e << ", ";
    std::cout << std::endl;
    std::cout << "Actual: ";
    for (const auto& a : act) std::cout << a << ", ";
    std::cout << std::endl;
  }
  return ret;
}

/* ************************************************************************* */
TEST(removeRepeats, no_repeats) {
  std::vector<double> a /**/ {1, 2, 3, 4, 5, 6, 7, 8, 9};
  std::vector<double> b /*  */ {9, 8, 7, 6, 5, 4, 3, 2};
  auto a_expected = a;
  auto b_expected = b;
  auto [a_end, b_end] = removeRepeats(a.begin(), a.end(), b.begin(), b.end());
  EXPECT(a.end() == a_end);
  EXPECT(b.end() == b_end);
  EXPECT(equals_print(a_expected, a));
  EXPECT(equals_print(b_expected, b));
}

/* ************************************************************************* */
TEST(removeRepeats, beginning_repeats_a) {
  std::vector<double> a /**/ {1, 1, 1, 1, 5, 6, 7, 8, 9};
  std::vector<double> b /*  */ {9, 8, 7, 6, 5, 4, 3, 2};
  std::vector<double> a_exp{1, 5, 6, 7, 8, 9};
  std::vector<double> b_exp{6, 5, 4, 3, 2};
  auto [a_end, b_end] = removeRepeats(a.begin(), a.end(), b.begin(), b.end());
  a.erase(a_end, a.end());
  b.erase(b_end, b.end());
  EXPECT(equals_print(a_exp, a));
  EXPECT(equals_print(b_exp, b));
}

/* ************************************************************************* */
TEST(removeRepeats, beginning_repeats_b) {
  std::vector<double> a /**/ {1, 2, 3, 4, 5, 6, 7, 8, 9};
  std::vector<double> b /*  */ {9, 9, 7, 6, 5, 4, 3, 2};
  std::vector<double> a_exp{1, 3, 4, 5, 6, 7, 8, 9};
  std::vector<double> b_exp{9, 7, 6, 5, 4, 3, 2};
  auto [a_end, b_end] = removeRepeats(a.begin(), a.end(), b.begin(), b.end());
  a.erase(a_end, a.end());
  b.erase(b_end, b.end());
  EXPECT(equals_print(a_exp, a));
  EXPECT(equals_print(b_exp, b));
}

/* ************************************************************************* */
TEST(removeRepeats, end_repeats_a) {
  std::vector<double> a /**/ {1, 2, 3, 4, 5, 6, 9, 9, 9};
  std::vector<double> b /*  */ {9, 8, 7, 6, 5, 4, 3, 2};
  std::vector<double> a_exp{1, 2, 3, 4, 5, 6, 9};
  std::vector<double> b_exp{9, 8, 7, 6, 5, 4};
  auto [a_end, b_end] = removeRepeats(a.begin(), a.end(), b.begin(), b.end());
  a.erase(a_end, a.end());
  b.erase(b_end, b.end());
  EXPECT(equals_print(a_exp, a));
  EXPECT(equals_print(b_exp, b));
}

/* ************************************************************************* */
TEST(removeRepeats, end_repeats_b) {
  std::vector<double> a /**/ {1, 2, 3, 4, 5, 6, 7, 8, 9};
  std::vector<double> b /*  */ {9, 8, 7, 6, 5, 2, 2, 2};
  std::vector<double> a_exp{1, 2, 3, 4, 5, 6, 9};
  std::vector<double> b_exp{9, 8, 7, 6, 5, 2};
  auto [a_end, b_end] = removeRepeats(a.begin(), a.end(), b.begin(), b.end());
  a.erase(a_end, a.end());
  b.erase(b_end, b.end());
  EXPECT(equals_print(a_exp, a));
  EXPECT(equals_print(b_exp, b));
}

/* ************************************************************************* */
TEST(removeRepeats, combined) {
  std::vector<double> a /**/ {1, 1, 1, 2, 3, 4, 5, 5, 5, 6, 7, 8, 8, 9};
  std::vector<double> b /*  */ {9, 9, 8, 8, 7, 6, 5, 4, 3, 2, 2, 2, 2};
  std::vector<double> a_exp{1, 3, 4, 5, 6, 9};
  std::vector<double> b_exp{8, 7, 6, 3, 2};
  auto [a_end, b_end] = removeRepeats(a.begin(), a.end(), b.begin(), b.end());
  a.erase(a_end, a.end());
  b.erase(b_end, b.end());
  EXPECT(equals_print(a_exp, a));
  EXPECT(equals_print(b_exp, b));
}

/* ************************************************************************* */
TEST(removeRepeats, combined2) {
  std::vector<double> a /**/ {1, 2, 3, 4, 5, 5, 5, 6, 7, 8, 9, 10, 11, 12, 13};
  std::vector<double> b /*  */ {9, 9, 8, 8, 7, 6, 8, 8, 5, 4, 3., 2., 2., 2.};
  std::vector<double> a_exp{1, 3, 7, 8, 9, 10, 13};
  std::vector<double> b_exp{9, 8, 5, 4, 3, 2};
  auto [a_end, b_end] = removeRepeats(a.begin(), a.end(), b.begin(), b.end());
  a.erase(a_end, a.end());
  b.erase(b_end, b.end());
  EXPECT(equals_print(a_exp, a));
  EXPECT(equals_print(b_exp, b));
}

/* ************************************************************************* */
TEST(removeRepeats, combined_chain) {
  std::vector<double> a /**/ {1, 1, 2, 1, 1, 4, 9, 9, 6, 1, 1, 8, 1, 1};
  std::vector<double> b /*  */ {9, 8, 8, 7, 8, 8, 5, 8, 8, 3, 8, 8, 1};
  std::vector<double> a_exp{1};
  std::vector<double> b_exp{};
  auto [a_end, b_end] = removeRepeats(a.begin(), a.end(), b.begin(), b.end());
  a.erase(a_end, a.end());
  b.erase(b_end, b.end());
  EXPECT(equals_print(a_exp, a));
  EXPECT(equals_print(b_exp, b));
}

/* ************************************************************************* */
TEST(removeRepeats, combined_chain2) {
  std::vector<double> a /**/ {1, 1, 2, 3, 3, 4, 5, 5, 6, 3, 3, 2, 1, 1};
  std::vector<double> b /*  */ {9, 8, 8, 7, 8, 8, 5, 8, 8, 3, 8, 8, 1};
  std::vector<double> a_exp{1};
  std::vector<double> b_exp{};
  auto [a_end, b_end] = removeRepeats(a.begin(), a.end(), b.begin(), b.end());
  a.erase(a_end, a.end());
  b.erase(b_end, b.end());
  EXPECT(equals_print(a_exp, a));
  EXPECT(equals_print(b_exp, b));
}

/* ************************************************************************* */
TEST(removeRepeats, custom_equals) {
  std::vector<double> a /**/ {1, 1.1, 2, 3.1, 3, 4, 5.1, 5, 6, 3.1, 3, 2, 1, 1};
  std::vector<double> b /*  */ {9, 8, 8.1, 7, 8, 8.1, 5, 8, 8.1, 3, 8.1, 8, 1};
  std::vector<double> a_exp{1};
  std::vector<double> b_exp{};
  auto custom_equals = [](double a, double b) {
    return std::abs(a - b) < 0.11;
  };
  auto [a_end, b_end] = removeRepeats(a.begin(), a.end(), b.begin(), b.end(),
                                      custom_equals, custom_equals);
  a.erase(a_end, a.end());
  b.erase(b_end, b.end());
  EXPECT(equals_print(a_exp, a));
  EXPECT(equals_print(b_exp, b));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
