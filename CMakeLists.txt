cmake_minimum_required(VERSION 3.14)
project(mavsim-viewer C)

set(CMAKE_C_STANDARD 11)

option(BUILD_TESTING "Build tests" OFF)

# MAVLink c_library_v2 (header-only, submodule)
set(MAVLINK_DIR ${CMAKE_SOURCE_DIR}/lib/c_library_v2)

# Core library: ULog parser + replay (no raylib dependency)
add_library(mavsim-core STATIC
    src/ulog_parser.c
    src/ulog_replay.c
    src/data_source_ulog.c
)
target_include_directories(mavsim-core PUBLIC src/)
target_include_directories(mavsim-core PRIVATE
    ${MAVLINK_DIR}
    ${MAVLINK_DIR}/common
)
target_compile_definitions(mavsim-core PRIVATE MAVLINK_COMM_NUM_BUFFERS=16)
if(UNIX AND NOT APPLE)
    target_link_libraries(mavsim-core PRIVATE m)
endif()

# Raylib via FetchContent (skip when only building tests)
if(NOT BUILD_TESTING_ONLY)
    include(FetchContent)
    FetchContent_Declare(
        raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG        5.5
        GIT_SHALLOW    TRUE
    )
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(raylib)

    add_executable(mavsim-viewer
        src/main.c
        src/mavlink_receiver.c
        src/data_source_mavlink.c
        src/vehicle.c
        src/scene.c
        src/hud.c
        src/hud_help.c
        src/hud_instruments.c
        src/hud_transport.c
        src/hud_telemetry.c
        src/replay_conflict.c
        src/replay_trail.c
        src/replay_markers.c
        src/ui_marker_input.c
        src/debug_panel.c
        src/ortho_panel.c
        src/asset_path.c
        src/theme.c
    )

    target_include_directories(mavsim-viewer PRIVATE
        ${MAVLINK_DIR}
        ${MAVLINK_DIR}/common
    )

    # Need 16 MAVLink parse channels for multi-vehicle support
    target_compile_definitions(mavsim-viewer PRIVATE MAVLINK_COMM_NUM_BUFFERS=16)

    target_link_libraries(mavsim-viewer PRIVATE raylib mavsim-core)

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
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/textures $<TARGET_FILE_DIR:mavsim-viewer>/textures
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/themes $<TARGET_FILE_DIR:mavsim-viewer>/themes
    )

    # Install rules
    include(GNUInstallDirs)

    target_compile_definitions(mavsim-viewer PRIVATE
        MAVSIM_INSTALL_DATADIR="${CMAKE_INSTALL_FULL_DATADIR}/mavsim-viewer")

    install(TARGETS mavsim-viewer RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    install(DIRECTORY models/ DESTINATION ${CMAKE_INSTALL_DATADIR}/mavsim-viewer/models
            FILES_MATCHING PATTERN "*.obj" PATTERN "*.mtl")
    install(DIRECTORY shaders/ DESTINATION ${CMAKE_INSTALL_DATADIR}/mavsim-viewer/shaders
            FILES_MATCHING PATTERN "*.vs" PATTERN "*.fs")
    install(DIRECTORY fonts/ DESTINATION ${CMAKE_INSTALL_DATADIR}/mavsim-viewer/fonts
            FILES_MATCHING PATTERN "*.ttf")
    install(DIRECTORY textures/ DESTINATION ${CMAKE_INSTALL_DATADIR}/mavsim-viewer/textures
            FILES_MATCHING PATTERN "*.png")
    install(DIRECTORY themes/ DESTINATION ${CMAKE_INSTALL_DATADIR}/mavsim-viewer/themes
            FILES_MATCHING PATTERN "*.mvt")
    install(FILES fonts/OFL.txt DESTINATION ${CMAKE_INSTALL_DATADIR}/mavsim-viewer/fonts)

    # CPack packaging
    set(CPACK_PACKAGE_NAME "mavsim-viewer")
    set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "3D MAVLink vehicle viewer for SITL simulation")
    set(CPACK_PACKAGE_CONTACT "https://github.com/mavlink/mavsim-viewer")
    set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")
    set(CPACK_DEBIAN_PACKAGE_DEPENDS
        "libgl1, libx11-6, libxrandr2, libxinerama1, libxcursor1, libxi6")
    set(CPACK_DEBIAN_PACKAGE_SECTION "science")
    set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
    if(DEFINED MAVSIM_VERSION)
        set(CPACK_PACKAGE_VERSION "${MAVSIM_VERSION}")
    endif()
    include(CPack)
endif()

if(BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()
