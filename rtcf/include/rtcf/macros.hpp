#ifndef MACROS_HPP
#define MACROS_HPP

// These macros are used to disable the OROCOS header warnings.

// clang-format off
#define OROCOS_HEADERS_BEGIN \
  _Pragma("GCC diagnostic push") \
  _Pragma("GCC diagnostic ignored \"-Wignored-qualifiers\"") \
  _Pragma("GCC diagnostic ignored \"-Wunused-parameter\"") \
// clang-format on

// clang-format off
#define OROCOS_HEADERS_END \
  _Pragma("GCC diagnostic pop")
// clang-format on

#endif  // MACROS_HPP
