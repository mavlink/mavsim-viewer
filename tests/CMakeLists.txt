set(FIXTURES_DIR ${CMAKE_SOURCE_DIR}/tests/fixtures)

# Tests use assert() for both setup and checks — ensure NDEBUG is never set
# so asserts are always active, even in Release builds.
remove_definitions(-DNDEBUG)

# ULog parser tests
add_executable(test_ulog_parser test_ulog_parser.c)
target_link_libraries(test_ulog_parser PRIVATE mavsim-core)
target_compile_definitions(test_ulog_parser PRIVATE FIXTURES_DIR="${FIXTURES_DIR}")
target_compile_options(test_ulog_parser PRIVATE -UNDEBUG)
add_test(NAME ulog_parser COMMAND test_ulog_parser)

# ULog replay tests
add_executable(test_ulog_replay test_ulog_replay.c)
target_link_libraries(test_ulog_replay PRIVATE mavsim-core)
target_compile_definitions(test_ulog_replay PRIVATE FIXTURES_DIR="${FIXTURES_DIR}")
target_compile_options(test_ulog_replay PRIVATE -UNDEBUG)
add_test(NAME ulog_replay COMMAND test_ulog_replay)

# Data source integration tests
add_executable(test_data_source test_data_source.c)
target_link_libraries(test_data_source PRIVATE mavsim-core)
target_compile_definitions(test_data_source PRIVATE FIXTURES_DIR="${FIXTURES_DIR}")
target_compile_options(test_data_source PRIVATE -UNDEBUG)
add_test(NAME data_source COMMAND test_data_source)

# OBJ model validation (catches missing texcoords that crash Raylib)
add_executable(test_obj_models test_obj_models.c)
target_compile_definitions(test_obj_models PRIVATE MODELS_DIR="${CMAKE_SOURCE_DIR}/models")
add_test(NAME obj_models COMMAND test_obj_models)
