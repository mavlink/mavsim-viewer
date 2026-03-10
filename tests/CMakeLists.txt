set(FIXTURES_DIR ${CMAKE_SOURCE_DIR}/tests/fixtures)

# ULog parser tests
add_executable(test_ulog_parser test_ulog_parser.c)
target_link_libraries(test_ulog_parser PRIVATE mavsim-core)
target_compile_definitions(test_ulog_parser PRIVATE FIXTURES_DIR="${FIXTURES_DIR}")
add_test(NAME ulog_parser COMMAND test_ulog_parser)

# ULog replay tests
add_executable(test_ulog_replay test_ulog_replay.c)
target_link_libraries(test_ulog_replay PRIVATE mavsim-core)
target_compile_definitions(test_ulog_replay PRIVATE FIXTURES_DIR="${FIXTURES_DIR}")
add_test(NAME ulog_replay COMMAND test_ulog_replay)

# Data source integration tests
add_executable(test_data_source test_data_source.c)
target_link_libraries(test_data_source PRIVATE mavsim-core)
target_compile_definitions(test_data_source PRIVATE FIXTURES_DIR="${FIXTURES_DIR}")
add_test(NAME data_source COMMAND test_data_source)
