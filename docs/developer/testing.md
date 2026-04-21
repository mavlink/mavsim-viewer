# Testing

Hawkeye uses [CTest](https://cmake.org/cmake/help/latest/manual/ctest.1.html) for its unit and integration tests.
Tests live under `tests/` and cover the core algorithms, data sources, ULog parser, replay logic, theme engine, UI logic, and OBJ model loader.

## Running the full test suite

```sh
make test
```

This builds Hawkeye with `-DBUILD_TESTING=ON` and runs every test under `ctest`.
Failures print full output (`--output-on-failure`).

## Core tests only (fast path)

For CI and quick iteration where you don't want to pull in raylib:

```sh
make test-core
```

This enables `-DBUILD_TESTING_ONLY=ON`, skipping the rendering stack and building only the headless core tests.
Much faster than a full build.

## Sanitizer build

To catch memory errors and undefined behavior:

```sh
make sanitize
```

This enables AddressSanitizer and UndefinedBehaviorSanitizer (`-fsanitize=address,undefined`), along with `-fno-omit-frame-pointer` for useful stack traces on failure.
Run the resulting binary through the test suite or your usual workflow; sanitizer output is printed to stderr when issues are detected.

## Test files

| File                    | Coverage                                                     |
| ----------------------- | ------------------------------------------------------------ |
| `test_algorithms.c`     | CUSUM takeoff detection, Pearson correlation, RMSE            |
| `test_data_source.c`    | Live/replay source abstraction, message dispatch             |
| `test_obj_models.c`     | Wavefront OBJ loader, texcoord validation                    |
| `test_theme.c`          | `.mvt` theme parser, palette lookup                          |
| `test_ui_logic.c`       | HUD state, vehicle selection, marker handling                |
| `test_ulog_parser.c`    | ULog file format parsing                                     |
| `test_ulog_replay.c`    | Replay transport, interpolation, seek                        |

Test fixtures are under `tests/fixtures/`.

## Integration testing with PX4 SITL

`tests/swarm_test.py` is a MAVSDK-based integration test that orchestrates a five-vehicle SITL swarm: it sets spawn offsets, arms all vehicles, takes off, flies a waypoint, and lands.
Not run by `make test` (requires PX4 SITL instances to be running); see [Live SITL Integration](../sitl.md#mavsdk-swarm-test-script) for how to run it.

## Running a single test under CTest

```sh
cd build
ctest -R algorithms --output-on-failure
```

The `-R <regex>` flag restricts to matching test names.
