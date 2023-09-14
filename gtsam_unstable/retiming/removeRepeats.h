#pragma once

#include "assert.h"

#include <iostream>

namespace gtsam {

// template <typename IteratorA, typename IteratorB>
// inline std::pair<IteratorA, IteratorB> helper(IteratorA& a_begin,
//                                               const IteratorA& a_last,
//                                               IteratorB& b_begin,
//                                               const IteratorB& b_last,
//                                               const IteratorA& a_begin_,
//                                               const IteratorB& b_begin_) {
//   static int count = 0;
//   static std::string indent = "";

//   indent += "  ";
//   // printf("%sRunning on %.0f - %.0f      %.0f - %.0f\n", indent.c_str(),
//   //        *a_begin, *a_last, *b_begin, *b_last);
//   printf("%sRunning on %d - %d      %d - %d,  %d\n", indent.c_str(),
//          std::distance(a_begin_, a_begin), std::distance(a_begin_, a_last),
//          std::distance(b_begin_, b_begin), std::distance(b_begin_, b_last),
//          (indent.length() % 4 == 0));

//   auto a_it = a_last;
//   auto b_it = b_last;
//   --a_it;
//   while ((a_it != a_begin) && (b_it != b_begin)) {
//     ++count;
//     if (count > 100) {
//       std::cout << "count: " << count << std::endl;
//       if (indent.length() > 1) indent.resize(indent.length() - 2);
//       return {a_begin, b_begin};
//     }

//     if (*a_it == *a_last) {
//       --a_it;
//       --b_it;
//     } else {
//       auto [b_write_head, a_write_head] =
//           helper(b_begin, b_it, a_begin, a_it, b_begin_, a_begin_);
//       if (*a_it != *a_begin) {
//         *(b_write_head++) = *b_it;
//         std::cout << indent << "Returning from here" << std::endl;
//         if (indent.length() > 1) indent.resize(indent.length() - 2);
//         return {a_write_head, b_write_head};
//       }
//     }
//   }
//   // assertm((a_it == a_last) && (b_it == b_last),
//   //         "a should be 1-element longer than b");
//   if (b_it != b_begin) {
//     *(b_begin++) = *b_it;
//     std::cout << indent << "Returning" << std::endl;
//     if (indent.length() > 1) indent.resize(indent.length() - 2);
//     return {a_begin, b_begin + 1};
//   }
//   std::cout << indent << "Returning" << std::endl;
//   if (indent.length() > 1) indent.resize(indent.length() - 2);
//   return {a_begin, b_begin};
// }

template <typename Iterator>
bool defaultEquals(const typename Iterator::reference& a,
                   const typename Iterator::reference& b) {
  return a == b;
}

template <typename Iterator>
using MyEquals = std::function<bool(const typename Iterator::reference&,
                                    const typename Iterator::reference&)>;

template <typename IteratorA, typename IteratorB>
inline std::pair<IteratorA, IteratorB> helper(
    const IteratorA& a_begin, const IteratorA& a_end, const IteratorB& b_begin,
    const IteratorB& b_end, const MyEquals<IteratorA>& a_equals) {
  auto a_it = a_begin + 1, a_write = a_begin + 1;
  auto b_it = b_begin, b_write = b_begin;
  while ((a_it != a_end) && (b_it != b_end)) {
    if (!a_equals(*a_it, *(a_write - 1))) {
      *(a_write++) = *a_it;
      *(b_write++) = *b_it;
    }
    ++a_it;
    ++b_it;
  }
  assertm(a_it == a_end, "a should be 1-element longer than b");
  assertm(b_it == b_end, "a should be 1-element longer than b");
  return {a_write, b_write};
}

/// Like std::remove, except removes the pair when a matches its next
/// element.
///
/// (repeats is when):
/// a:       a   b   b   c   d   d   d   d   e
/// b:         x   y   z   z   x   y   z   x
///              ^^^   vvv   ^^^^^^^^^^^
/// Notice:
///  1. a should be 1-element longer than b
///  2. We remove the first element in the pair when a has a match, but the next
///  element when b has a match
template <typename IteratorA, typename IteratorB>
inline std::pair<IteratorA, IteratorB> removeRepeats(
    const IteratorA& a_begin, const IteratorA& a_end, const IteratorB& b_begin,
    const IteratorB& b_end,
    const MyEquals<IteratorA>& a_equals = defaultEquals<IteratorA>,
    const MyEquals<IteratorB>& b_equals = defaultEquals<IteratorB>) {
  IteratorA a_result = a_end;
  IteratorB b_result = b_end;
  IteratorA a_prev_result = a_begin;
  IteratorB b_prev_result = b_begin;
  while ((a_result != a_prev_result) || (b_result != b_prev_result)) {
    a_prev_result = a_result;
    b_prev_result = b_result;

    auto [a_write1, b_write1] =
        helper(a_begin, a_result, b_begin, b_result, a_equals);
    a_result = a_write1;
    b_result = b_write1;
    if (std::distance(a_begin, a_write1) > 2) {
      auto [b_write2, a_write2] =
          helper(b_begin, b_result, a_begin + 1, a_result - 1, b_equals);
      *a_write2 = *(a_result - 1);
      a_result = a_write2 + 1;
      b_result = b_write2;
    }
  }

  return {a_result, b_result};

  // auto a_it = a_begin;
  // auto b_it = b_begin;

  // something with a

  // auto [a_new, b_new] = helper(b_it, b_end, a_it, a_end);
  // if (*a_result == *a_new) {
  // }

  // auto a_result = a_begin, a_it = a_begin, a_compare = a_begin;
  // auto b_result = b_begin, b_it = b_begin, b_compare = b_begin;

  // if ((a_begin == a_end) || (b_begin == b_end)) return {a_end, b_end};
  // auto a_end_less1 = a_end - 1;
  // auto b_end_less1 = b_end - 1;
  // auto done = [&]() { return (a_it == a_end_less1) || (b_it == b_end_less1);
  // };

  // // At the start, we are certain of the first element in a
  // ++a_result;
  // bool comparing_a = true;

  // while (!done()) {
  //   if (comparing_a) {  // a is ahead of b
  //     if (*a_compare == *a_it) {
  //       ++a_it;
  //       ++b_it;
  //     } else {
  //       if (*b_it == *b_compare) {
  //         // Backtrack!
  //         a_compare = a_result;
  //         ++b_it;
  //         comparing_a = false;
  //       } else {
  //         *(a_result++) = *a_compare;  // finalize a_compare
  //         comparing_a = false;
  //       }
  //       b_compare = b_it;
  //       *b_result = *b_it;  // tentative result
  //       ++b;
  //       comparing_a = false;
  //     }
  //   } else {  // b is ahead of a
  //     if (*b_result == *b_it) {
  //       ++a_it;
  //       ++b_it;
  //     } else {
  //       if (*a_it == *a_compare) {
  //         ++a_it;
  //         comparing_a = true;
  //       } else {
  //         *(b_result++) = *b_compare;  // finalize b_compare
  //         comparing_a = true;
  //       }
  //     }
  //   }
  // }
  // assertm(a_it != a_end, "a should be 1-element longer than b");
  // if (*a_it != *a_result) {
  //   *(++a_result) = *a_it;
  // } else {
  //   --b_result;
  // }
  // ++a_it;
  // assertm(a_it == a_end, "a should be 1-element longer than b");
  // ++a_result;
  // ++b_result;
  // return {a_result, b_result};
}
}  // namespace gtsam
