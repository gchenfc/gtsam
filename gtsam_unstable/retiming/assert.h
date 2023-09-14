/**
 * @file assert.h
 * @author Gerry Chen
 * @brief Custom assertion for runtime tests
 * @date Sept 2023
 */

#pragma once

#include <stdexcept>

namespace gtsam {

// Use (void) to silence unused warnings.
// #ifdef NDEBUG
// #define assertm(exp, msg) (void)(exp)  // expression may have side effects
// #else
// #define assertm(exp, msg) assert(((void)msg, exp))
// #endif
#define assertm(exp, msg)                      \
  {                                            \
    if (!(exp)) throw std::runtime_error(msg); \
  }

}  // namespace gtsam
