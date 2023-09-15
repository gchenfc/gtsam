set(excluded_examples "benchmark_toppra.cpp")
gtsamAddExamplesGlob("*.cpp" "${excluded_examples}" "gtsam_unstable")

set(BUILD_TOPPRA_BENCHMARK OFF)
if (BUILD_TOPPRA_BENCHMARK)
  find_package(toppra)
  gtsamAddExamplesGlob("benchmark_toppra.cpp" "" "gtsam;toppra::toppra")
endif()
