set(EXCLUDE_TESTS "${EXCLUDE_TESTS}")

gtsamAddTestsGlob(retiming_unstable "test*.cpp" "${EXCLUDE_TESTS}" "gtsam_unstable")
