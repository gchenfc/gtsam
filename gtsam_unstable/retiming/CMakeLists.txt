# Install headers
file(GLOB retiming_headers "*.h")
install(FILES ${retiming_headers} DESTINATION include/gtsam_unstable/retiming)

# Add all tests
add_subdirectory(tests)
