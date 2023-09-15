set(excluded_examples "benchmark_toppra.cpp")
gtsamAddExamplesGlob("*.cpp" "${excluded_examples}" "gtsam_unstable")

option(BUILD_TOPPRA_BENCHMARK "BUILD TOPPRA" OFF)
if (BUILD_TOPPRA_BENCHMARK)
  find_package(toppra)
  gtsamAddExamplesGlob("benchmark_toppra.cpp" "" "gtsam;toppra::toppra")
endif()
