BUILD_TYPE ?= Debug
BUILD_DIR  := build

# Cross-platform parallel job count
ifeq ($(OS),Windows_NT)
    JOBS := $(NUMBER_OF_PROCESSORS)
    RM   := rmdir /s /q
    EXE  := $(BUILD_DIR)\mavsim-viewer.exe
else
    JOBS := $(shell sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)
    RM   := rm -rf
    EXE  := $(BUILD_DIR)/mavsim-viewer
endif

.PHONY: build configure test clean release run

build: configure
	cmake --build $(BUILD_DIR) --config $(BUILD_TYPE) -j$(JOBS)

configure:
	cmake -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) -DBUILD_TESTING=ON

test: build
	ctest --test-dir $(BUILD_DIR) --output-on-failure -C $(BUILD_TYPE)

release:
	$(MAKE) BUILD_TYPE=Release

clean:
	$(RM) $(BUILD_DIR)

run: build
	$(EXE)
