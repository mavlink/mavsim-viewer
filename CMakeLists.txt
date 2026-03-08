cmake_minimum_required(VERSION 3.14)
project(mavsim-viewer C)

set(CMAKE_C_STANDARD 11)

# Raylib via FetchContent
include(FetchContent)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG        5.5
    GIT_SHALLOW    TRUE
)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(raylib)

# MAVLink c_library_v2 (header-only, submodule)
set(MAVLINK_DIR ${CMAKE_SOURCE_DIR}/lib/c_library_v2)

add_executable(mavsim-viewer
    src/main.c
    src/mavlink_receiver.c
    src/vehicle.c
    src/scene.c
    src/hud.c
    src/debug_panel.c
)

target_include_directories(mavsim-viewer PRIVATE
    ${MAVLINK_DIR}
    ${MAVLINK_DIR}/common
)

# Need 16 MAVLink parse channels for multi-vehicle support
target_compile_definitions(mavsim-viewer PRIVATE MAVLINK_COMM_NUM_BUFFERS=16)

target_link_libraries(mavsim-viewer PRIVATE raylib)

if(APPLE)
    target_link_libraries(mavsim-viewer PRIVATE
        "-framework IOKit"
        "-framework Cocoa"
        "-framework OpenGL"
    )
elseif(WIN32)
    target_link_libraries(mavsim-viewer PRIVATE ws2_32)
elseif(UNIX)
    target_link_libraries(mavsim-viewer PRIVATE m pthread dl)
endif()

# Copy resources to build directory
add_custom_command(TARGET mavsim-viewer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/models $<TARGET_FILE_DIR:mavsim-viewer>/models
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/shaders $<TARGET_FILE_DIR:mavsim-viewer>/shaders
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/fonts $<TARGET_FILE_DIR:mavsim-viewer>/fonts
)
